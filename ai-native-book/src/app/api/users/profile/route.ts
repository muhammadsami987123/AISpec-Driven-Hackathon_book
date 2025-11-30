import { NextRequest, NextResponse } from "next/server";
import jwt from "jsonwebtoken";
import { getUserByEmail, updateUserInQdrant } from "@/lib/qdrant";
import { profileUpdateSchema } from "@/lib/validation";

export async function GET(req: NextRequest) {
  try {
    // Get token from cookie or header
    const token =
      req.cookies.get("authToken")?.value ||
      req.headers.get("authorization")?.replace("Bearer ", "");

    if (!token) {
      return NextResponse.json(
        { error: "Unauthorized" },
        { status: 401 }
      );
    }

    // Verify JWT token
    const decoded = jwt.verify(
      token,
      process.env.JWT_SECRET || "secret"
    ) as any;

    // Get user from Qdrant
    const user = await getUserByEmail(decoded.email);
    if (!user) {
      return NextResponse.json(
        { error: "User not found" },
        { status: 404 }
      );
    }

    return NextResponse.json(
      {
        user: {
          id: user.id,
          email: user.email,
          name: user.name,
          image: user.image,
          provider: user.provider,
          createdAt: user.createdAt,
        },
      },
      { status: 200 }
    );
  } catch (error: any) {
    console.error("Get user error:", error);

    if (error.name === "JsonWebTokenError") {
      return NextResponse.json(
        { error: "Invalid token" },
        { status: 401 }
      );
    }

    return NextResponse.json(
      { error: "Failed to fetch user" },
      { status: 500 }
    );
  }
}

export async function PUT(req: NextRequest) {
  try {
    // Get token from cookie or header
    const token =
      req.cookies.get("authToken")?.value ||
      req.headers.get("authorization")?.replace("Bearer ", "");

    if (!token) {
      return NextResponse.json(
        { error: "Unauthorized" },
        { status: 401 }
      );
    }

    // Verify JWT token
    const decoded = jwt.verify(
      token,
      process.env.JWT_SECRET || "secret"
    ) as any;

    const body = await req.json();

    // Validate input
    const validatedData = profileUpdateSchema.parse(body);

    // Update user in Qdrant
    const updatedUser = await updateUserInQdrant(decoded.userId, {
      name: validatedData.fullName || undefined,
      email: validatedData.email || undefined,
      image: validatedData.image || undefined,
    });

    return NextResponse.json(
      {
        message: "User profile updated successfully",
        user: updatedUser,
      },
      { status: 200 }
    );
  } catch (error: any) {
    console.error("Update user error:", error);

    if (error.name === "JsonWebTokenError") {
      return NextResponse.json(
        { error: "Invalid token" },
        { status: 401 }
      );
    }

    if (error.name === "ZodError") {
      return NextResponse.json(
        { error: "Validation failed", details: error.errors },
        { status: 400 }
      );
    }

    return NextResponse.json(
      { error: "Failed to update user" },
      { status: 500 }
    );
  }
}
