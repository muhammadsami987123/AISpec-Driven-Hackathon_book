# 3+1 Chapter Overview Layout — Technical Explanation

## Why 550px Cards Do Not Fit

The default Docusaurus container width is approximately **1140px wide**.

If you place 3 cards at **550px each**:
- 3 × 550px = **1650px total width**
- 1650px > 1140px → **cards cannot fit without wrapping**
- Result: automatic 2+1 layout (2 cards in row 1, 1 card in row 2)

By reducing card width to **400px** and increasing the container to **1280px**:
- 3 × 400px = **1200px**
- 1200px + gap spacing = fits within 1280px container
- Result: **perfect 3+1 layout** (3 cards in row 1, 1 centered card in row 2)

This is the only way to achieve a strict 3+1 layout without forcing wrapping or distorting the design.

## Layout Implementation

### Desktop (≥1200px)
- **Container width**: 1280px (strict)
- **Row 1**: 3 cards at 400px each with 40px gaps
- **Row 2**: 1 card centered under the middle card of row 1
- **No wrapping**: Cards maintain their fixed positions

### Tablet (768px–1199px)
- **Cards per row**: 2 (flexible width, max 400px)
- **Container**: Scales to fit available space (max ~900px)
- **Behavior**: Cards shrink gracefully to maintain 2-column layout

### Mobile (<768px)
- **Cards per row**: 1 (full width)
- **Container**: 100% width with side padding
- **Behavior**: Single-column stack, cards shrink to fit screen

## Component Architecture

### ChapterOverview.js
- **Vanilla React**: No UI libraries (pure React + Docusaurus Link)
- **Chapter data**: Array of 4 chapters with title, description, and link
- **Rendering**: Two flex rows—first with `slice(0,3)`, second with `slice(3,4)`
- **Docusaurus-compatible**: Uses Docusaurus `Link` component, compatible with v3

### ChapterOverview.module.css
- **Flexbox layout**: Flex rows with `justify-content: center` for alignment
- **Card dimensions**: 400px × 350px on desktop, responsive on smaller screens
- **Typography**: Serif title (Georgia), sans-serif description
- **Hover effect**: `transform: scale(1.05)` with enhanced box-shadow
- **Responsive breakpoints**: Media queries at 1200px (desktop), 768px (tablet), 767px (mobile)

## Key Constraints (All Met)

✅ Container is exactly **1280px** wide (ensures 3 × 400px fits)  
✅ Cards are exactly **400px × 350px** on desktop  
✅ 3+1 layout enforced—no 2+2 wrapping possible  
✅ Row 1 has exactly 3 cards, row 2 has exactly 1 centered card  
✅ No icons, no images—clean, text-focused design  
✅ Vanilla React + Docusaurus Link (no external UI libraries)  
✅ Builds successfully with Docusaurus v3  
✅ Responsive: tablet shows 2/row, mobile shows 1/row  
✅ Hover effect: `scale(1.05)` + shadow enhancement  
✅ Professional robotics-themed descriptions (3–4 lines each)  

## Files Delivered

1. **ChapterOverview.js** — Full React component with 4 chapters
2. **ChapterOverview.module.css** — Complete stylesheet with responsive breakpoints
3. **This explanation** — Technical rationale for layout choices

All files are ready for integration into Docusaurus v3 and are fully compatible with the Gemini CLI toolchain.
