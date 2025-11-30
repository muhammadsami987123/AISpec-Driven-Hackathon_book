import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

// --- Hero Section Component ---
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', 'heroBanner')}>
      <div className="container">
        <div className={clsx('row', 'heroSplit')}>
          <div className="col col--6">
            <img src="/img/hero-placeholder.jpg" alt="Physical AI & Humanoid Robotics Book Cover" className="bookCover" />
          </div>
          <div className="col col--6">
            <Heading as="h1" className="hero__title">
              Physical AI & Humanoid Robotics
            </Heading>
            <p className="hero__subtitle">
              Physical AI marks a transformative frontier in artificial intelligence, integrating AI directly with the physical world. This book explores intelligent systems that possess a physical form, perceive, interact with, and act within their environment. Discover the essence of embodiment, its distinction from traditional AI, and the advancements driving autonomous vehicles, humanoid robots, and industrial automation.
            </p>
            <div className="buttons">
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Continue Book
              </Link>
              <Link
                className="button button--info button--lg"
                to="#chapter-overview"> {/* Scrolls to chapter overview section */}
                Explore Topics
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

// --- Chapter Overview Section Component ---
const chapterData = [
  {
    title: 'Introduction to Physical AI',
    description: 'Defining embodied intelligence and its transformative impact.',
    link: '/docs/intro',
    icon: '/img/icons/robot.svg',
  },
  {
    title: 'Foundations of Humanoid Robotics',
    description: 'Understanding the mechanics and control of robotic systems.',
    link: '/docs/chapter-two',
    icon: '/img/icons/gears.svg',
  },
  {
    title: 'Advanced Embodied Intelligence',
    description: 'Exploring perception, learning, and human-robot interaction.',
    link: '/docs/chapter-three',
    icon: '/img/icons/brain.svg',
  },
  {
    title: 'Real-World Applications & Ethics',
    description: 'Deployment considerations, safety, and the future of physical AI.',
    link: '/docs/chapter-four',
    icon: '/img/icons/earth.svg',
  },
];

function ChapterOverviewSection() {
  return (
    <section id="chapter-overview" className={clsx('container', 'chapterOverviewSection')}>
      <Heading as="h2" className="text--center">
        Explore the Book's Core Topics
      </Heading>
      <div className={clsx('row', 'chapterGrid')}>
        {chapterData.map((chapter, idx) => (
          <div key={idx} className={clsx('col col--6 margin-bottom--lg', 'chapterCardWrapper')}>
            <Link to={chapter.link} className="chapterCard">
              <img src={chapter.icon} alt={chapter.title + " icon"} className="chapterIcon" /> {/* Using image placeholder */}
              <h3>{chapter.title}</h3>
              <p>{chapter.description}</p>
            </Link>
          </div>
        ))}
      </div>
    </section>
  );
}

// --- Testimonials Section Component (Existing) ---
const testimonials = [
  {
    quote: "An essential read for anyone interested in the future of AI. The concepts are explained with remarkable clarity!",
    author: "- Dr. Alex Tech, AI Researcher",
  },
  {
    quote: "This book transformed my understanding of embodied AI. Highly recommended for both beginners and experts.",
    author: "- Sarah Innovate, Robotics Engineer",
  },
  {
    quote: "Finally, a comprehensive guide to Physical AI that is both insightful and practical. A must-have!",
    author: "- Prof. J. Data, Computer Science Dept.",
  },
  {
    quote: "A groundbreaking work that bridges the gap between theoretical AI and its real-world physical applications.",
    author: "- Dr. Emily Robotica, Lead AI Scientist",
  },
];

function TestimonialsSection() {
  const [currentTestimonial, setCurrentTestimonial] = useState(0);

  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTestimonial((prev) => (prev + 1) % testimonials.length);
    }, 5000); // Change testimonial every 5 seconds
    return () => clearInterval(timer);
  }, []);

  const handleDotClick = (index: number) => {
    setCurrentTestimonial(index);
  };

  return (
    <section className="testimonialsSection">
      <div className="container">
        <Heading as="h2" className="text--center">
          What Readers Say
        </Heading>
        <div className="testimonialSlider">
          {testimonials.map((testimonial, index) => (
            <div
              key={index}
              className={clsx('testimonialItem', {
                activeTestimonial: index === currentTestimonial,
              })}
            >
              <p className="testimonialQuote">"{testimonial.quote}"</p>
              <p className="testimonialAuthor">{testimonial.author}</p>
            </div>
          ))}
        </div>
        <div className="testimonialDots">
          {testimonials.map((_, index) => (
            <span
              key={index}
              className={clsx('dot', {
                activeDot: index === currentTestimonial,
              })}
              onClick={() => handleDotClick(index)}
            ></span>
          ))}
        </div>
      </div>
    </section>
  );
}

// --- What Makes This Book Different Section Component ---
const featureCardsData = [
  {
    icon: 'fa-solid fa-brain',
    title: 'Deep AI Integration',
    description: 'Explore cutting-edge AI models directly applied to physical systems.',
  },
  {
    icon: 'fa-solid fa-robot',
    title: 'Humanoid Focus',
    description: 'Dedicated insights into the design, control, and intelligence of humanoid robots.',
  },
  {
    icon: 'fa-solid fa-cogs',
    title: 'Practical Applications',
    description: 'Case studies and real-world scenarios demonstrating physical AI in action.',
  },
  {
    icon: 'fa-solid fa-globe',
    title: 'Ethical Considerations',
    description: 'In-depth discussion on the societal and ethical impacts of advanced robotics.',
  },
  {
    icon: 'fa-solid fa-code',
    title: 'Software & Hardware',
    description: 'Covers both the AI software and the underlying robotic hardware architectures.',
  },
  {
    icon: 'fa-solid fa-book-open',
    title: 'Future Outlook',
    description: 'Forecasting the next decade of physical AI and humanoid robotics advancements.',
  },
];

function WhatMakesThisBookDifferentSection() {
  return (
    <section className="whatMakesThisBookDifferentSection">
      <div className="container">
        <Heading as="h2" className="text--center">
          What Makes This Book Different?
        </Heading>
        <div className={clsx('row', 'featureCardsGrid')}>
          {featureCardsData.map((card, idx) => (
            <div key={idx} className={clsx('col col--4 margin-bottom--lg', 'featureCardWrapper')}>
              <div className="featureCard">
                <i className={clsx(card.icon, 'featureIcon')}></i>
                <h3>{card.title}</h3>
                <p>{card.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// --- Main Home Component ---
export default function Home(): React.ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const chapterOverviewRef = useRef<HTMLElement>(null);

  // Function to scroll to the chapter overview section
  const scrollToChapterOverview = () => {
    chapterOverviewRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // This might be removed as the button now directly links to the anchor
  // But keeping it as a pattern if direct scroll logic is needed later
  useEffect(() => {
    if (window.location.hash === '#chapter-overview' && chapterOverviewRef.current) {
      scrollToChapterOverview();
    }
  }, []);


  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Physical AI marks a transformative frontier in artificial intelligence, integrating AI directly with the physical world. This book explores intelligent systems that possess a physical form, perceive, interact with, and act within their environment. Discover the essence of embodiment, its distinction from traditional AI, and the advancements driving autonomous vehicles, humanoid robots, and industrial automation.">
      <HomepageHeader />
      <main>
        <ChapterOverviewSection /> {/* New section */}
        <TestimonialsSection />
        <WhatMakesThisBookDifferentSection /> {/* New section */}
      </main>
    </Layout>
  );
}