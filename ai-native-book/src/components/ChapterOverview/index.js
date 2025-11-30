// src/components/ChapterOverview/index.js
import React from 'react';
import Link from '@docusaurus/Link';
import clsx from 'clsx'; // Utility for conditionally joining CSS class names
import styles from './ChapterOverview.module.css'; // Import the CSS module for scoped styles
import Heading from '@theme/Heading'; // Import Docusaurus Heading component for accessibility and styling

/**
 * chapterData: Defines the content for each chapter card.
 * Each object contains a heading, a detailed description, and a link to its corresponding documentation page.
 * Icons are explicitly excluded as per requirements. Descriptions are crafted to be 3-4 lines for presentation.
 */
const chapterData = [
  {
    heading: "Introduction to Physical AI",
    description: "Dive into the nascent field of Physical AI, exploring its foundational principles, historical evolution, and the critical distinction between symbolic and embodied intelligence. This chapter sets the stage for understanding how AI can learn and act in the real world through physical interaction, highlighting the transformative potential across robotics, automation, and human-computer symbiosis.",
    link: "/docs/chapter-1"
  },
  {
    heading: "Foundations of Humanoid Robotics",
    description: "Delve deep into the engineering marvels of humanoid robotics, covering advanced kinematics, dynamics, and control architectures. From servo-motor mechanics to compliant actuation, this chapter details the biological inspirations and mathematical models that enable robots to move, balance, and interact with complex environments, laying the groundwork for truly autonomous agents.",
    link: "/docs/chapter-2"
  },
  {
    heading: "Sensory Perception & State Estimation",
    description: "Unlock the secrets of how physical AI systems interpret the world around them. This chapter explores cutting-edge sensor technologies—vision, lidar, force/torque—and the sophisticated algorithms for data fusion, object recognition, and real-time state estimation. Understand how robots construct rich internal representations of their environment to navigate, manipulate, and anticipate changes.",
    link: "/docs/chapter-3"
  },
  {
    heading: "Locomotion & Control",
    description: "Master the intricate art and science of robot locomotion and fine-grained control. This chapter provides a comprehensive overview of dynamic walking, agile balancing strategies, and dexterous manipulation techniques. Explore reinforcement learning, optimal control, and adaptive strategies that enable physical AI to overcome unforeseen challenges and perform complex tasks in unstructured settings.",
    link: "/docs/chapter-4"
  }
];

/**
 * ChapterOverview Component:
 * Renders a responsive grid of chapter cards.
 * - Desktop: 3 cards in the first row, 1 centered card in the second row (3+1 layout).
 * - Tablet: 2 cards per row.
 * - Mobile: 1 card per row.
 * Adheres to strict width, spacing, and styling requirements.
 */
function ChapterOverview() {
  return (
    // Section directly under the Hero component, with Docusaurus margin utility classes
    <section className={clsx("margin-top--xl", "margin-bottom--xl")}>
      <div className="container text--center"> {/* Centered heading for the section */}
        <Heading as="h2">
          Explore the Book's Core Topics
        </Heading>
      </div>
      {/* The grid container that manages the layout of the chapter cards */}
      <div className={clsx(styles.chapterOverviewGrid)}>
        {chapterData.map((chapter, index) => (
          // Each card is a clickable Docusaurus Link component
          <Link to={chapter.link} className={clsx(styles.card)} key={index}>
            {/* Chapter Heading (serif font) */}
            <h3>{chapter.heading}</h3>
            {/* Chapter Description (sans-serif font) */}
            <p>{chapter.description}</p>
          </Link>
        ))}
      </div>
    </section>
  );
}

export default ChapterOverview;
