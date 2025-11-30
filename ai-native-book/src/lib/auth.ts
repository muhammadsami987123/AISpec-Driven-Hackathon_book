import NextAuth from "next-auth";
import GoogleProvider from "next-auth/providers/google";
import CredentialsProvider from "next-auth/providers/credentials";
import { QdrantClient } from "qdrant-js-client";
import bcryptjs from "bcryptjs";

const qdrantClient = new QdrantClient({
  url: process.env.NEXT_PUBLIC_QDRANT_URL || "",
  apiKey: process.env.QDRANT_API_KEY || "",
});

export const { handlers, auth, signIn, signOut } = NextAuth({
  providers: [
    GoogleProvider({
      clientId: process.env.GOOGLE_CLIENT_ID || "",
      clientSecret: process.env.GOOGLE_CLIENT_SECRET || "",
      allowDangerousEmailAccountLinking: true,
    }),
    CredentialsProvider({
      name: "Credentials",
      credentials: {
        email: { label: "Email", type: "email" },
        password: { label: "Password", type: "password" },
      },
      async authorize(credentials) {
        if (!credentials?.email || !credentials?.password) {
          throw new Error("Invalid email or password");
        }

        try {
          // Search for user in Qdrant
          const searchResults = await qdrantClient.search("users", {
            vector: [0], // Placeholder; actual implementation would hash credentials
            limit: 1,
            query_filter: {
              must: [
                {
                  key: "email",
                  match: {
                    value: credentials.email as string,
                  },
                },
              ],
            },
          });

          if (searchResults.result.length === 0) {
            throw new Error("User not found");
          }

          const user = searchResults.result[0].payload as any;

          // Verify password
          const passwordMatch = await bcryptjs.compare(
            credentials.password as string,
            user.passwordHash || ""
          );

          if (!passwordMatch) {
            throw new Error("Invalid password");
          }

          return {
            id: user.id,
            email: user.email,
            name: user.name,
            image: user.image,
          };
        } catch (error) {
          console.error("Authorization error:", error);
          throw new Error("Authentication failed");
        }
      },
    }),
  ],

  pages: {
    signIn: "/login",
    signUp: "/signup",
    error: "/auth-error",
  },

  callbacks: {
    async signIn({ user, account, profile }) {
      try {
        // Only store data for Google provider or successful credential auth
        if (account?.provider === "google" && profile) {
          // Check if user exists in Qdrant
          const existingUser = await qdrantClient.search("users", {
            vector: [0],
            limit: 1,
            query_filter: {
              must: [
                {
                  key: "email",
                  match: {
                    value: profile.email || user.email || "",
                  },
                },
              ],
            },
          });

          if (existingUser.result.length === 0) {
            // Store new user in Qdrant
            await storeUserInQdrant({
              email: profile.email || user.email || "",
              name: profile.name || user.name || "",
              image: profile.image || user.image || "",
              provider: "google",
              googleId: profile.sub,
            });
          }
        }

        return true;
      } catch (error) {
        console.error("Sign-in callback error:", error);
        return true; // Allow sign-in despite error to prevent blocking
      }
    },

    async jwt({ token, user, account }) {
      if (user) {
        token.id = user.id;
        token.email = user.email;
      }
      if (account) {
        token.provider = account.provider;
      }
      return token;
    },

    async session({ session, token }) {
      if (session.user) {
        session.user.id = token.id as string;
        session.user.email = token.email as string;
      }
      return session;
    },

    async redirect({ url, baseUrl }) {
      // Redirect to dashboard after login
      if (url.startsWith("/")) return `${baseUrl}/dashboard`;
      if (new URL(url).origin === baseUrl) return url;
      return baseUrl;
    },
  },

  session: {
    strategy: "jwt",
    maxAge: 7 * 24 * 60 * 60, // 7 days
  },

  events: {
    async signOut() {
      // Optional: cleanup operations
    },
  },
});

/**
 * Store user data in Qdrant vector database
 */
async function storeUserInQdrant(userData: {
  email: string;
  name: string;
  image?: string;
  provider: string;
  googleId?: string;
}) {
  try {
    const userId = `user_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    
    // Create a point with user data
    const userPoint = {
      id: userId,
      vector: [0], // Placeholder vector; use actual embeddings in production
      payload: {
        email: userData.email,
        name: userData.name,
        image: userData.image || null,
        provider: userData.provider,
        googleId: userData.googleId || null,
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
    };

    // Upsert user point to Qdrant
    await qdrantClient.upsert("users", {
      points: [userPoint],
    });

    console.log(`User ${userData.email} stored in Qdrant with ID: ${userId}`);
    return userId;
  } catch (error) {
    console.error("Error storing user in Qdrant:", error);
    throw error;
  }
}

export default auth;
