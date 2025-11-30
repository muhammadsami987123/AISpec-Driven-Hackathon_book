import { NextRequest, NextResponse } from "next/server";
import { loginSchema } from "@/lib/validation";
import { getUserByEmail, initializeUsersCollection } from "@/lib/qdrant";
import bcryptjs from "bcryptjs";
import jwt from "jsonwebtoken";

export async function POST(req: NextRequest) {
  try {
    // Initialize collection on first request
    await initializeUsersCollection();

    const body = await req.json();

    // Validate input
    const validatedData = loginSchema.parse(body);

    // Find user by email
    const user = await getUserByEmail(validatedData.email);
    if (!user) {
      return NextResponse.json(
        { error: "Invalid email or password" },
        { status: 401 }
      );
    }

    // Verify password
    const passwordMatch = await bcryptjs.compare(
      validatedData.password,
      user.passwordHash || ""
    );
    if (!passwordMatch) {
      return NextResponse.json(
        { error: "Invalid email or password" },
        { status: 401 }
      );
    }

    // Generate JWT token
    const token = jwt.sign(
      {
        userId: user.id,
        email: user.email,
        name: user.name,
      },
      process.env.JWT_SECRET || "secret",
      {
        expiresIn: process.env.JWT_EXPIRATION || "7d",
      }
    );

    // Create response with secure cookie
    const response = NextResponse.json(
      {
        message: "Login successful",
        token,
        user: {
          id: user.id,
          email: user.email,
          name: user.name,
          image: user.image,
        },
      },
      { status: 200 }
    );

    // Set secure HTTP-only cookie
    response.cookies.set("authToken", token, {
      httpOnly: true,
      secure: process.env.NODE_ENV === "production",
      sameSite: "lax",
      maxAge: 7 * 24 * 60 * 60, // 7 days
      path: "/",
    });

    return response;
  } catch (error: any) {
    console.error("Login error:", error);

    if (error.name === "ZodError") {
      return NextResponse.json(
        { error: "Validation failed", details: error.errors },
        { status: 400 }
      );
    }

    return NextResponse.json(
      { error: "Login failed. Please try again." },
      { status: 500 }
    );
  }
}
