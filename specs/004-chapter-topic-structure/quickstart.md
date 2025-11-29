# Quickstart: Creating a New Chapter Topic File

This guide outlines the process for creating a new topic file within an existing chapter, adhering to the established structure and quality requirements.

## Steps:

1.  **Navigate to the Chapter Directory**:
    *   Locate the target chapter's directory within the `ai-native-book/docs` structure. For example, for Chapter Four, this would be `ai-native-book/docs/04-chapter-four/`.

2.  **Create a New Markdown File**:
    *   Inside the chapter's directory, create a new Markdown file (`.md`). The filename should be descriptive, lowercase, and use hyphens for spaces (e.g., `understanding-llms.md`). This file will serve as a sibling to the `index.md` and other topic files in that directory.

3.  **Populate with Content**:
    *   Open the newly created `.md` file.
    *   Add a suitable title using an H1 heading (`# Your Topic Title`).
    *   Write the complete, detailed, page-level content for the topic. Ensure it is research-based, contextually aligned with the chapter and overall book theme, and provides depth, examples, and explanations.
    *   Maintain the consistent writing style and professional formatting observed in previous chapters.

4.  **Add to Docusaurus Sidebar (Optional but Recommended)**:
    *   To make the new topic discoverable and navigable via the Docusaurus sidebar, you will typically need to update the `sidebars.ts` file in the `ai-native-book/` directory.
    *   Locate the entry for your chapter and add your new topic file's path (relative to the `docs` directory) to the items list.
    *   Example `sidebars.ts` entry for Chapter Four with a new topic:
        ```typescript
        {
          type: 'category',
          label: 'Chapter 4: Language-Guided Reasoning',
          items: [
            '04-chapter-four/index', // Main chapter overview
            '04-chapter-four/understanding-llms', // Your new topic file
            // ... other topic files for Chapter 4
          ],
        },
        ```
    *   The `sidebar_label` can be specified directly in the `sidebars.ts` entry or by adding front matter to your `.md` file (e.g., `---sidebar_label: Understanding LLMs---`).

5.  **Review Content Quality**:
    *   Perform a manual review of the new topic file to ensure it meets all writing quality requirements (FR-005, FR-006, FR-007, FR-008, FR-009) and the success criteria (SC-003, SC-004) from the `spec.md`.

This process ensures that new topic files are correctly integrated into the Docusaurus structure and maintain the desired content quality.
