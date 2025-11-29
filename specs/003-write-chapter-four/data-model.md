# Data Model: Chapter Content

## Entity: Chapter

### Description
Represents a chapter in the book.

### Fields
-   **title** (string): The title of the chapter.
-   **content** (markdown): The full content of the chapter in Markdown format.
-   **order** (integer): The numerical order of the chapter in the book.

### Relationships
-   A **Book** has many **Chapters**.
