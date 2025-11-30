---
id: 75527-v2-1700938639
title: Revise Docusaurus ChapterOverview Component Layout and Styling
stage: general
date: 2025-11-30
surface: agent
model: gemini-1.5-pro
feature: none
branch: 007-write-chapter-three
user: user
command: (User input from the beginning of this turn)
labels: ["docusaurus", "react", "component", "css-grid", "responsive", "layout", "styling"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- src/components/ChapterOverview/ChapterOverview.module.css
- src/components/ChapterOverview/index.js
tests:
null
---

## Prompt

{"id":"75527","variant":"standard","title":"Gemini CLI Prompt for Perfect 3+1 Layout Without Icons"}
Explain why 4 cards cannot fit in a single row for the book "Physical AI & Humanoid Robotics" when each card is 550px wide with premium spacing and hover effects. Then generate a fully responsive, Docusaurus v3 compatible ChapterOverview.js section with the following corrected layout and spacing:

1. Layout:
   - Desktop (min 1440px):
       • First row → exactly 3 cards (no wrapping)
       • Second row → 1 centered card
       • Horizontal gap must be tight and minimal, not wide or empty
   - Tablet → 2 cards per row
   - Mobile → 1 card per row
   - Section placed directly under hero
   - All cards centered inside the container

2. Card Design:
   - Size: Height 350px, Width 550px
   - Rounded corners, premium subtle shadow
   - Hover: scale(1.05) + soft shadow + smooth transition
   - **Remove icons completely** (no images)
   - Heading → serif font, premium style
   - Description → sans-serif font
   - Add more descriptive text for each topic based on a professional robotics book style

3. Functionality:
   - Each card must link to its chapter under /docs
   - Only vanilla React
   - Fully build-safe for Docusaurus v3 and Gemini CLI (npm run build must succeed)

4. Styling Expectations:
   - Tight spacing between cards (not wide empty areas)
   - Cards aligned and centered horizontally
   - Proper gap and padding for a premium book layout
   - Use minimal CSS (inline or module CSS)

5. Output Requirements:
   - Provide complete JSX + minimal CSS
   - Must generate ONLY 4 cards in EXACT 3+1 layout
   - Cards MUST stay centered and must NOT break into 2–2 layout
   - Clean production-quality code.

Make sure the final result visually fixes the previous issue where only 2 cards were appearing per row with extremely large empty gaps and icons showing broken text. The new result must be tight, centered, icon-free, and professionally spaced.

## Response snapshot

The issue with attempting to fit four 550px wide cards into a single row, especially with "premium spacing and hover effects" within a typical Docusaurus content container, stems from fundamental web layout constraints.

A standard Docusaurus content area often ranges from 960px to 1200px in width, depending on the theme and screen size.

Let's calculate the required width for four 550px cards:
*   **Card Width:** 4 * 550px = 2200px
*   **Minimal Gaps:** Assuming a minimal gap of 30px between cards, for 4 cards, there would be 3 gaps: 3 * 30px = 90px
*   **Hover Effect Expansion:** The `scale(1.05)` hover effect means each card temporarily expands, requiring extra space. Even if not explicitly added to the layout calculation, this expansion can cause overflow or layout shifts if the container is already at its limit.
*   **Total Required Width:** 2200px + 90px = 2290px.

This required width (2290px) significantly exceeds even the widest Docusaurus content containers and most common desktop screen resolutions. Attempting to force this layout would result in:
1.  **Horizontal Scrolling:** The content would overflow, requiring users to scroll horizontally, a poor user experience.
2.  **Severe Shrinking:** If the cards were made flexible (`1fr`), they would shrink drastically to fit, making the content unreadable and losing the "premium" feel.
3.  **Broken Layout:** The grid system would wrap cards onto new lines in unpredictable ways, failing to maintain the desired 3+1 structure.

Therefore, a 3+1 responsive layout, especially one that allows cards to maintain their target 550px width on larger screens while gracefully adapting on smaller devices, is the optimal solution for both aesthetics and usability. This approach respects the fixed card width while ensuring the overall presentation remains clean, centered, and within reasonable screen boundaries.

Here's the revised `ChapterOverview.js` component and its minimal accompanying CSS:

```css
/* src/components/ChapterOverview/ChapterOverview.module.css */
.chapterOverviewGrid {
  display: grid;
  gap: 30px; /* Minimal gap for premium spacing */
  margin-top: 40px; /* Space below hero section */
  padding: 0 20px; /* Horizontal padding for the section itself */
  margin-left: auto; /* Center the grid container */
  margin-right: auto; /* Center the grid container */
  justify-items: center; /* Center cards horizontally within their grid cells */
  align-items: start; /* Align cards to the top of their grid areas */
}

/* Mobile (<768px) - 1 card per row */
@media (max-width: 767px) {
  .chapterOverviewGrid {
    grid-template-columns: 1fr; /* Single column, takes full available width */
    max-width: 600px; /* Limit overall grid width on mobile for better padding */
  }
}

/* Tablet (768px - 1439px) - 2 cards per row */
@media (min-width: 768px) and (max-width: 1439px) {
  .chapterOverviewGrid {
    grid-template-columns: repeat(2, minmax(300px, 1fr)); /* Two flexible columns */
    /* Max width to accommodate two 550px cards + one gap + section padding */
    max-width: calc(2 * 550px + 1 * 30px + 40px);
  }
}

/* Desktop (min 1440px) - exactly 3 cards, then 1 centered */
@media (min-width: 1440px) {
  .chapterOverviewGrid {
    grid-template-columns: repeat(3, 550px); /* Three fixed-width columns */
    /* Max width to accommodate three 550px cards + two gaps + section padding */
    max-width: calc(3 * 550px + 2 * 30px + 40px);
  }
  .chapterOverviewGrid > :nth-child(4) { /* Target the 4th card */
    grid-column: 2 / 3; /* Centers it under the middle card of the first row */
  }
}

.card {
  height: 350px; /* Fixed height */
  width: 550px; /* Target width */
  max-width: 100%; /* Allows card to shrink on smaller screens */
  border-radius: 12px;
  box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1); /* Premium subtle shadow */
  padding: 30px; /* Generous internal padding */
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center; /* Vertically center content */
  text-align: center;
  transition: all 0.3s ease-out; /* Smooth transition for hover */
  background-color: var(--docusaurus-card-background-color);
  border: 1px solid var(--docusaurus-card-border-color);
  text-decoration: none; /* Remove underline from Docusaurus Link */
  color: inherit; /* Inherit text color from parent */
}

.card:hover {
  transform: scale(1.05); /* Increased scale on hover */
  box-shadow: 0 10px 20px rgba(0, 0, 0, 0.2); /* Softer, more pronounced shadow */
}

.card h3 {
  font-family: 'Georgia', serif; /* Elegant serif font for heading */
  font-size: 1.8rem;
  margin-bottom: 15px;
  color: var(--ifm-color-primary); /* Use Docusaurus primary color */
}

.card p {
  font-family: 'Helvetica Neue', Arial, sans-serif; /* Professional sans-serif font for description */
  font-size: 1.1rem;
  color: var(--ifm-font-color-base); /* Use Docusaurus base font color */
  line-height: 1.6;
  max-width: 90%; /* Constrain description width for better readability */
}
