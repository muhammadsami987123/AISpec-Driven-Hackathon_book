# Data Model: Chapter Topic File

## Entity: TopicFile

### Description
Represents an individual topic file (`.md`) within a chapter's directory in a Docusaurus project.

### Fields
-   **filename** (string): The name of the markdown file (e.g., `introduction-to-llms.md`). Should be descriptive and URL-friendly.
-   **content** (markdown): The complete, detailed, page-level content of the topic, including headings, text, code blocks, and other markdown elements.
-   **chapter_directory** (string): The path to the parent chapter's directory (e.g., `ai-native-book/docs/04-chapter-four`).
-   **sidebar_label** (string, optional): The label that will appear in the Docusaurus sidebar navigation for this topic.

### Relationships
-   A **Chapter Directory** contains many **TopicFiles**.
-   **TopicFiles** are linked and ordered within the Docusaurus `sidebars.ts` configuration.
