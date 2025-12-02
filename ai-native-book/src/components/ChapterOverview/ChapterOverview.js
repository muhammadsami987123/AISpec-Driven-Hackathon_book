import React from 'react';
import Link from '@docusaurus/Link';
import styles from './ChapterOverview.module.css';

/**
 * ChapterOverview Component
 * 
 * Displays a 3+1 card layout for the Physical AI & Humanoid Robotics book chapters.
 * - Desktop (≥1200px): 3 cards in row 1, 1 centered card in row 2
 * - Tablet (768px–1199px): 2 cards per row
 * - Mobile (<768px): 1 card per row
 * 
 * Container is 1280px wide to ensure 3 × 400px cards fit without wrapping.
 */
export default function ChapterOverview() {
  const chapters = [
    {
      id: 'chapter1',
      title: 'Foundations of Physical AI',
      description: 'Explore the foundational concepts of Physical AI, embodied intelligence, and the perception-decision-action loop that powers modern humanoid systems.',
      link: '/docs/01-chapter-one/',
    },
    {
      id: 'chapter2',
      title: 'Building and Deploying Humanoid Robots',
      description: 'Master advanced robot architectures, hardware integration, control systems, and the practical challenges of deploying humanoids in real-world environments.',
      link: '/docs/02-chapter-two',
    },
    {
      id: 'chapter3',
      title: 'Perception and Sensor Fusion',
      description: 'Dive into computer vision, lidar processing, and multi-sensor fusion techniques that enable robots to understand and interact with their surroundings.',
      link: '/docs/03-chapter-three',
    },
    {
      id: 'chapter4',
      title: 'Learning and Adaptation',
      description: 'Learn reinforcement learning, imitation learning, and sim-to-real transfer methods that allow robots to acquire new skills and improve over time.',
      link: '/docs/04-chapter-four',
    },
  ];

  return (
    <section className={styles.chapterOverviewSection}>
      <div className={styles.container}>
        {/* Row 1: First 3 cards */}
        <div className={styles.cardRow}>
          {chapters.slice(0, 3).map((chapter) => (
            <Link key={chapter.id} to={chapter.link} className={styles.cardLink}>
              <div className={styles.card}>
                <h3 className={styles.cardTitle}>{chapter.title}</h3>
                <p className={styles.cardDescription}>{chapter.description}</p>
              </div>
            </Link>
          ))}
        </div>

        {/* Row 2: 4th card (centered) */}
        <div className={styles.cardRowSingle}>
          {chapters.slice(3, 4).map((chapter) => (
            <Link key={chapter.id} to={chapter.link} className={styles.cardLink}>
              <div className={styles.card}>
                <h3 className={styles.cardTitle}>{chapter.title}</h3>
                <p className={styles.cardDescription}>{chapter.description}</p>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}
