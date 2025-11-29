# Data Model: Chapter 2 Topic File

## Entity: Chapter2TopicFile

### Description
Represents an individual topic file (`.md`) within the `02-chapter-two` directory in a Docusaurus project.

### Fields
-   **filename** (string): The name of the markdown file (e.g., `kinematics-dynamics.md`). Should be descriptive, lowercase, and URL-friendly.
-   **title** (string): The primary title of the topic within the file (e.g., "Humanoid Robot Kinematics and Dynamics").
-   **content** (markdown): The complete, detailed, page-level content of the topic, adhering to the 800-1200 word count, including headings, text, code blocks, examples, explanations, and professional formatting.
-   **chapter_directory** (string): The fixed path to the parent chapter's directory (`ai-native-book/docs/02-chapter-two/`).
-   **sidebar_label** (string, optional): The label that will appear in the Docusaurus sidebar navigation for this topic.

### Relationships
-   The `ai-native-book/docs/02-chapter-two/` directory contains many **Chapter2TopicFiles**.
-   **Chapter2TopicFiles** are linked and ordered within the Docusaurus `sidebars.ts` configuration (likely via autogeneration).
