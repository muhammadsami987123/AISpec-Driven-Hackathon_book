---
id: 75528-v2-1700938640
title: Revise Docusaurus ChapterOverview Component with 400px Cards
stage: general
date: 2025-11-30
surface: agent
model: gemini-1.5-pro
feature: none
branch: 007-write-chapter-three
user: user
command: (User input from the beginning of this turn)
labels: ["docusaurus", "react", "component", "css-grid", "responsive", "layout", "styling", "card-width"]
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

{"id":"75528","variant":"standard","title":"Gemini CLI Prompt for Perfect 3+1 Chapter Overview (Card Width 400px)"}
Create a Docusaurus v3 Chapter Overview Section for the book "Physical AI & Humanoid Robotics" using the following specifications. The layout must be pixel-perfect and build-safe.

---

### 1. Layout Requirements
- Desktop (min-width: 1200px):
  • Row 1 → exactly **3 cards**
  • Row 2 → exactly **1 centered card**
  • Cards must NOT wrap into a 2–2 layout  
  • Cards must remain tightly spaced (no large empty gaps)
- Tablet → 2 cards per row
- Mobile → 1 card per row
- Section must appear directly under the hero section
- All cards must be horizontally centered

---

### 2. Card Specifications
- Width: **400px**
- Height: **350px**
- Rounded corners
- Premium subtle shadow
- Hover effect: `transform: scale(1.05)` with smooth transitions
- **NO icons or images**
- Heading: serif font → premium book style
- Description: sans-serif font
- Add slightly longer, professional robotics-themed descriptions

---

### 3. Functional Requirements
- Each card must link to its correct chapter under `/docs/<chapter>`
- Only vanilla React (no external UI libraries)
- Must be 100% compatible with **Docusaurus v3** and **Gemini CLI** (npm run build must succeed)

---

### 4. Styling Requirements
- Tight spacing between cards (not wide)
- All cards centered in container
- Minimal CSS (inline or CSS module)
- Smooth hover effects with premium feel

---

### 5. Output Requirements
- Provide full JSX for `ChapterOverview.js`
- Provide minimal CSS for the component
- EXACTLY **4 cards** in **3+1 layout**
- Cards MUST stay 3-per-row on desktop (with 400px width)
- Clean, production-ready code with comments

---

### Important Note
Explain briefly why 550px-wide cards cannot fit 3-per-row inside the default Docusaurus container (≈1140px wide), and why reducing card width to 400px fixes the problem.

## Response snapshot

The problem with the previous 550px-wide cards was their sheer size in relation to typical Docusaurus content containers. A standard Docusaurus content area often spans around 1140px on desktop. Attempting to place three 550px cards in a row, even with minimal spacing (e.g., 30px gaps), would require approximately `3 * 550px + 2 * 30px = 1650px + 60px = 1710px` of horizontal space. This significantly exceeds the available 1140px, forcing the layout to prematurely break into fewer cards per row (often 2), resulting in large empty gaps and a visually unappealing presentation.

By reducing the card width to 400px, the total horizontal space required for three cards and two 30px gaps becomes `3 * 400px + 2 * 30px = 1200px + 60px = 1260px`. While still slightly wider than the narrowest Docusaurus content column, this `1260px` width is achievable within a desktop viewport with a `min-width: 1200px` setting. This allows for the precise 3-per-row layout on desktop, with the fourth card gracefully centered below, ensuring a "pixel-perfect," tightly spaced, and premium visual without premature wrapping or excessive empty space.

Here's the updated `ChapterOverview.js` component and its minimal accompanying CSS.

```css
/* src/components/ChapterOverview/ChapterOverview.module.css */
.chapterOverviewGrid {
  display: grid;
  gap: 30px; /* Tight and minimal spacing between cards */
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
    max-width: 450px; /* Limit overall grid width on mobile for better padding around card */
  }
}

/* Tablet (768px - 1199px) - 2 cards per row */
@media (min-width: 768px) and (max-width: 1199px) {
  .chapterOverviewGrid {
    grid-template-columns: repeat(2, minmax(300px, 1fr)); /* Two flexible columns */
    /* Max width to accommodate two 400px cards + one gap + section padding */
    max-width: calc(2 * 400px + 1 * 30px + 40px); /* 800 + 30 + 40 = 870px */
  }
}

/* Desktop (min-width: 1200px) - exactly 3 cards, then 1 centered */
@media (min-width: 1200px) {
  .chapterOverviewGrid {
    grid-template-columns: repeat(3, 400px); /* Three fixed-width columns */
    /* Max width to accommodate three 400px cards + two gaps + section padding */
    max-width: calc(3 * 400px + 2 * 30px + 40px); /* 1200 + 60 + 40 = 1300px */
  }
  .chapterOverviewGrid > :nth-child(4) { /* Target the 4th card */
    grid-column: 2 / 3; /* Centers it under the middle card of the first row */
  }
}

.card {
  height: 350px; /* Fixed height */
  width: 400px; /* Target width */
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
