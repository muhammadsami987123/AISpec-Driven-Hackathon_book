import { NextRequest, NextResponse } from "next/server";
import bcryptjs from "bcryptjs";
import { signupSchema } from "@/lib/validation";
import {
  getUserByEmail,
  storeUserInQdrant,
  initializeUsersCollection,
} from "@/lib/qdrant";

export async function POST(req: NextRequest) {
  try {
    // Initialize collection on first request
    await initializeUsersCollection();

    const body = await req.json();

    // Validate input
    const validatedData = signupSchema.parse(body);

    // Check if user already exists
    const existingUser = await getUserByEmail(validatedData.email);
    if (existingUser) {
      return NextResponse.json(
        { error: "Email already registered" },
        { status: 409 }
      );
    }

    // Hash password
    const passwordHash = await bcryptjs.hash(validatedData.password, 10);

    // Store user in Qdrant
    const userId = await storeUserInQdrant({
      email: validatedData.email,
      name: validatedData.fullName,
      provider: "credentials",
      passwordHash,
    });

    // Return success response
    return NextResponse.json(
      {
        message: "User registered successfully",
        userId,
        email: validatedData.email,
      },
      { status: 201 }
    );
  } catch (error: any) {
    console.error("Signup error:", error);

    if (error.name === "ZodError") {
      return NextResponse.json(
        { error: "Validation failed", details: error.errors },
        { status: 400 }
      );
    }

    return NextResponse.json(
      { error: "Signup failed. Please try again." },
      { status: 500 }
    );
  }
}
