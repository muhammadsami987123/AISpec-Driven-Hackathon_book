import { QdrantClient } from "qdrant-js-client";
import crypto from "crypto";

const qdrantClient = new QdrantClient({
  url: process.env.NEXT_PUBLIC_QDRANT_URL || "",
  apiKey: process.env.QDRANT_API_KEY || "",
});

/**
 * Initialize Qdrant collection for users if it doesn't exist
 */
export async function initializeUsersCollection() {
  try {
    const collections = await qdrantClient.getCollections();

    const userCollectionExists = collections.collections?.some(
      (col: any) => col.name === "users"
    );

    if (!userCollectionExists) {
      await qdrantClient.createCollection("users", {
        vectors: {
          size: 1, // Placeholder size
          distance: "Cosine",
        },
      });
      console.log("Users collection created in Qdrant");
    }
  } catch (error) {
    console.error("Error initializing Qdrant collection:", error);
    throw error;
  }
}

/**
 * Get user by email from Qdrant
 */
export async function getUserByEmail(email: string) {
  try {
    const searchResults = await qdrantClient.search("users", {
      vector: [0],
      limit: 1,
      query_filter: {
        must: [
          {
            key: "email",
            match: {
              value: email,
            },
          },
        ],
      },
    });

    if (searchResults.result.length === 0) {
      return null;
    }

    return {
      id: searchResults.result[0].id,
      ...searchResults.result[0].payload,
    };
  } catch (error) {
    console.error("Error fetching user by email:", error);
    throw error;
  }
}

/**
 * Store user in Qdrant
 */
export async function storeUserInQdrant(userData: {
  email: string;
  name: string;
  image?: string;
  provider: string;
  googleId?: string;
  passwordHash?: string;
}) {
  try {
    const userId = `user_${crypto.randomBytes(8).toString("hex")}`;

    const userPoint = {
      id: userId,
      vector: [0],
      payload: {
        email: userData.email,
        name: userData.name,
        image: userData.image || null,
        provider: userData.provider,
        googleId: userData.googleId || null,
        passwordHash: userData.passwordHash || null,
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
      },
    };

    await qdrantClient.upsert("users", {
      points: [userPoint],
    });

    console.log(`User ${userData.email} stored in Qdrant`);
    return userId;
  } catch (error) {
    console.error("Error storing user in Qdrant:", error);
    throw error;
  }
}

/**
 * Update user in Qdrant
 */
export async function updateUserInQdrant(
  userId: string,
  updates: Partial<any>
) {
  try {
    const existingUser = await qdrantClient.retrieve("users", {
      ids: [userId],
    });

    if (!existingUser || existingUser.length === 0) {
      throw new Error("User not found");
    }

    const updatedPayload = {
      ...existingUser[0].payload,
      ...updates,
      updatedAt: new Date().toISOString(),
    };

    await qdrantClient.upsert("users", {
      points: [
        {
          id: userId,
          vector: [0],
          payload: updatedPayload,
        },
      ],
    });

    return updatedPayload;
  } catch (error) {
    console.error("Error updating user in Qdrant:", error);
    throw error;
  }
}

/**
 * Delete user from Qdrant
 */
export async function deleteUserFromQdrant(userId: string) {
  try {
    await qdrantClient.delete("users", {
      points_selector: {
        points: [userId],
      },
    });

    console.log(`User ${userId} deleted from Qdrant`);
  } catch (error) {
    console.error("Error deleting user from Qdrant:", error);
    throw error;
  }
}

/**
 * Search users by criteria (advanced search)
 */
export async function searchUsers(filter: any, limit: number = 10) {
  try {
    const results = await qdrantClient.search("users", {
      vector: [0],
      limit,
      query_filter: filter,
    });

    return results.result.map((r: any) => ({
      id: r.id,
      ...r.payload,
    }));
  } catch (error) {
    console.error("Error searching users:", error);
    throw error;
  }
}

export { qdrantClient };
