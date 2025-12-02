import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

// --- Types for structured chapter metadata ---
type ChapterTag =
  | 'Foundations'
  | 'Robotics Stack'
  | 'Simulation'
  | 'Learning'
  | 'Language & Reasoning'
  | 'Ethics & Governance'
  | 'Capstone';

type ChapterMeta = {
  id: string;
  order: string;
  title: string;
  description: string;
  link: string;
  icon: string;
  tags: ChapterTag[];
  highlights: string[];
};

const chapters: ChapterMeta[] = [
  {
    id: 'chapter-01-intro',
    order: 'Chapter 1',
    title: 'Foundations of Physical AI & Course Spec',
    description:
      'Define Physical AI, embodied intelligence, and the full capstone roadmap from sensors to humanoid deployment.',
    link: '/docs/01-chapter-one/',
    // Use existing static illustrations for chapter icons
    icon: '/img/undraw_docusaurus_tree.svg',
    tags: ['Foundations', 'Robotics Stack', 'Ethics & Governance'],
    highlights: [
      'Historical context & definition of Physical AI',
      'Perception–Decision–Action loop',
      'Quarter-long roadmap & prerequisites',
    ],
  },
  {
    id: 'chapter-02-humanoids',
    order: 'Chapter 2',
    title: 'Building & Deploying Humanoid Robots',
    description:
      'Architect, sense, and control humanoid systems operating in human environments with robust perception and control.',
    link: '/docs/02-chapter-two',
    icon: '/img/undraw_docusaurus_mountain.svg',
    tags: ['Robotics Stack', 'Simulation', 'Capstone'],
    highlights: [
      'Advanced robot architectures & whole‑body control',
      'Perception and multi-sensor fusion',
      'Control systems for bipedal locomotion',
    ],
  },
  {
    id: 'chapter-03-advanced',
    order: 'Chapter 3',
    title: 'Advanced Learning, HRI & Ethics',
    description:
      'Dive into reinforcement learning, human–robot interaction, and the ethical landscape of embodied AI.',
    link: '/docs/03-chapter-three',
    icon: '/img/undraw_docusaurus_react.svg',
    tags: ['Learning', 'Ethics & Governance'],
    highlights: [
      'Reinforcement & imitation learning for robots',
      'Human–robot interaction principles',
      'Ethical considerations in robotics',
    ],
  },
  {
    id: 'chapter-04-lgr',
    order: 'Chapter 4',
    title: 'Language‑Guided Reasoning (LGR)',
    description:
      'Connect large language models, perception, and ROS 2 to turn natural language into robot action.',
    link: '/docs/04-chapter-four',
    icon: '/img/logo.svg',
    tags: ['Language & Reasoning', 'Robotics Stack', 'Capstone'],
    highlights: [
      'NLU, planning, and symbolic grounding',
      'Integrating LLMs with ROS 2 and semantic maps',
      'End‑to‑end VLA agents and future challenges',
    ],
  },
];

const chapterFilters: { id: ChapterTag | 'All'; label: string }[] = [
  { id: 'All', label: 'All topics' },
  { id: 'Foundations', label: 'Foundations' },
  { id: 'Robotics Stack', label: 'Robotics Stack' },
  { id: 'Simulation', label: 'Simulation & Digital Twins' },
  { id: 'Learning', label: 'Robot Learning' },
  { id: 'Language & Reasoning', label: 'Language‑Guided Reasoning' },
  { id: 'Ethics & Governance', label: 'Ethics & Governance' },
  { id: 'Capstone', label: 'Capstone Path' },
];

// --- HERO SECTION ---
function HomepageHero() {
  // Use path without leading slash to respect baseUrl, and add a safe fallback
  const heroImg = useBaseUrl('img/book-cover.png');
  const fallbackImg = useBaseUrl('img/new.png');
  return (
    <header className={clsx('hero hero--primary', 'heroBanner')}>
      <div className="container">
        <div className="heroSplit">
          <div className="heroText">
            <Heading as="h1" className="hero__title">
              The AI-Native Guide to <br />
              <span className="textHighlight">Physical AI & Humanoid Robotics</span>
            </Heading>
            <p className="hero__subtitle">
              Master the convergence of artificial intelligence, robotics, and embodied intelligence. Learn to build intelligent physical systems through hands-on, spec-driven development with industry-standard tools like ROS 2, Gazebo, and modern AI.
            </p>
            <div className="hero__badges">
              <span className="heroBadge">Spec-Driven</span>
              <span className="heroBadge">AI-Native</span>
              <span className="heroBadge">ROS 2 & Gazebo</span>
              <span className="heroBadge">Humanoid Control</span>
            </div>
            <div className="buttons">
              <Link className="button button--secondary button--lg" to="/docs/introduction/">
                Continue Book
              </Link>
              <Link className="button button--info button--lg" to="#book-journey">
                Explore Topics
              </Link>
            </div>
          </div>
          <div className="heroImage">
            <img
              src={heroImg}
              alt="Physical AI & Humanoid Robotics book cover"
              className="bookCover"
              onError={(e) => {
                // Fallback to a known-good image if the main asset is unavailable
                (e.currentTarget as HTMLImageElement).src = fallbackImg;
              }}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

// --- QUICK VALUE/USP SECTION ---
function ValuePropsSection() {
  const props = [
    {
      icon: '/img/undraw_docusaurus_mountain.svg',
      headline: 'Real Capstone Robot',
      text: 'Build, simulate, and deploy an autonomous humanoid. Not a toy project—industry stack, real skills.'
    },
    {
      icon: '/img/undraw_docusaurus_tree.svg',
      headline: 'Spec-Driven Learning',
      text: 'Your progress is driven by living specifications, AI‑powered labs, and real code tests.'
    },
    {
      icon: '/img/undraw_docusaurus_react.svg',
      headline: 'AI-Native Labs',
      text: 'Work with modern LLMs, vision, and digital twins. From perception to control, language, and safety.'
    }
  ];
  const resolve = useBaseUrl;
  return (
    <section className="valuePropsSection">
      <div className="container">
        <div className="valuePropsGrid">
          {props.map((p, idx) => (
            <div key={idx} className="valuePropCard">
              <img
                src={resolve(p.icon)}
                alt={p.headline + " icon"}
                className="valuePropIcon"
              />
              <h3>{p.headline}</h3>
              <p>{p.text}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// --- BOOK ROADMAP SECTION ---
function TimelineSection() {
  const timeline = [
    { title: 'Foundations', desc: 'Physical AI, robotics landscape, prerequisites, core ethics.' },
    { title: 'ROS 2 & Perception', desc: 'Sensors, fusion, robot nervous system, core comms.' },
    { title: 'Simulation & Digital Twins', desc: 'High-fidelity Gazebo/Isaac prototyping.' },
    { title: 'AI Perception & Planning', desc: 'Vision, object recognition, SLAM, path planning.' },
    { title: 'Full Capstone & Real Robots', desc: 'Manipulation, language, autonomy deployment.' }
  ];
  return (
    <section id="book-journey" className="timelineSection">
      <div className="container">
        <Heading as="h2" className="text--center sectionTitle">Your Journey At a Glance</Heading>
        <div className="timelineGrid">
          {timeline.map((step, i) =>
            <div key={i} className="timelineCard">
              <h3>{step.title}</h3>
              <p>{step.desc}</p>
            </div>
          )}
        </div>
      </div>
    </section>
  );
}

// --- CHAPTERS SECTION (RESPONSIVE GRID) ---
function ChapterOverviewSection() {
  const [activeFilter, setActiveFilter] = useState('All');
  const chapterFilters = [
    { id: 'All', label: 'All topics' },
    { id: 'Foundations', label: 'Foundations' },
    { id: 'Robotics Stack', label: 'Robotics Stack' },
    { id: 'Simulation', label: 'Simulation & Digital Twins' },
    { id: 'Learning', label: 'Robot Learning' },
    { id: 'Language & Reasoning', label: 'Language‑Guided Reasoning' },
    { id: 'Ethics & Governance', label: 'Ethics & Governance' },
    { id: 'Capstone', label: 'Capstone Path' },
  ];
  const filteredChapters = activeFilter === 'All' ? chapters : chapters.filter(ch => ch.tags.includes(activeFilter));
  return (
    <section className="chapterOverviewSection">
      <div className="container">
        <Heading as="h2" className="text--center sectionTitle">Explore All Chapters & Topics</Heading>
        <div className="chapterFilters">
          {chapterFilters.map(filter => (
            <button key={filter.id} className={clsx('chapterFilterChip', activeFilter === filter.id && 'chapterFilterChip--active')} type="button" onClick={() => setActiveFilter(filter.id)}>{filter.label}</button>
          ))}
        </div>
        <div className="chapterGrid">
          {filteredChapters.map((chapter, idx) => (
            <Link to={chapter.link} className="chapterCard" key={chapter.id}>
              <div className="chapterCardHeader">
                <span className="chapterOrder">{chapter.order}</span>
                <img src={useBaseUrl(chapter.icon)} alt={chapter.title + " icon"} className="chapterIcon" />
              </div>
              <h3>{chapter.title}</h3>
              <p>{chapter.description}</p>
              <div className="chapterHighlights">
                {chapter.highlights.map(item => (<span key={item} className="chapterHighlightPill">{item}</span>))}
              </div>
              <div className="chapterTagRow">
                {chapter.tags.map(tag => (<span key={tag} className="chapterTagPill">{tag}</span>))}
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

// --- FEATURE GRID SECTION ---
function WhatMakesThisBookDifferentSection() {
  const features = [
    { icon: '/img/undraw_docusaurus_tree.svg', text: 'Spec-driven, AI-powered workflow where docs are code, and labs always match your real robot progress.' },
    { icon: '/img/undraw_docusaurus_mountain.svg', text: 'Everything proven in simulation before real hardware. Safe, industry-standard stack.' },
    { icon: '/img/undraw_docusaurus_react.svg', text: 'Language models, perception, ROS 2: you build a complete, world-ready stack.' },
    { icon: '/img/undraw_docusaurus_tree.svg', text: 'Learning-enabled—Reinforcement/Imitation directly connect to simulation and perception.' },
    { icon: '/img/undraw_docusaurus_mountain.svg', text: 'Ethics and lab governance are never an afterthought: trust and safety by design.' },
    { icon: '/img/undraw_docusaurus_react.svg', text: 'Humanoids as grand challenge: bipedal walking, hands, vision, language, autonomy.' },
  ];
  const resolve = useBaseUrl;
  return (
    <section className="whatMakesThisBookDifferentSection">
      <div className="container">
        <Heading as="h2" className="text--center sectionTitle">What Makes This Book Different?</Heading>
        <div className="featureCardsGrid">
          {features.map((card, idx) => (
            <div key={idx} className="featureCard">
              <img src={resolve(card.icon)} alt="Feature icon" className="featureIcon" />
              <p>{card.text}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// --- TESTIMONIALS SECTION ---
const testimonials = [
  { quote: "An essential read for anyone interested in the future of AI. The concepts are explained with remarkable clarity!", author: "- Dr. Alex Tech, AI Researcher" },
  { quote: "This book transformed my understanding of embodied AI. Highly recommended for both beginners and experts.", author: "- Sarah Innovate, Robotics Engineer" },
  { quote: "Finally, a comprehensive guide to Physical AI that is both insightful and practical. A must-have!", author: "- Prof. J. Data, Computer Science Dept." },
  { quote: "A groundbreaking work that bridges the gap between theoretical AI and its real-world physical applications.", author: "- Dr. Emily Robotica, Lead AI Scientist" },
];
function TestimonialsSection() {
  const [current, setCurrent] = useState(0);
  useEffect(() => { const timer = setInterval(() => setCurrent((p) => (p + 1) % testimonials.length), 6500); return () => clearInterval(timer); }, []);
  return (
    <section className="testimonialsSection">
      <div className="container">
        <Heading as="h2" className="text--center sectionTitle">What Readers Say</Heading>
        <div className="testimonialSlider">
          {testimonials.map((t, i) => (
            <div key={i} className={clsx('testimonialItem', { activeTestimonial: i === current })}>
              <p className="testimonialQuote">"{t.quote}"</p>
              <p className="testimonialAuthor">{t.author}</p>
            </div>
          ))}
        </div>
        <div className="testimonialDots">
          {testimonials.map((_, i) => (
            <span key={i} className={clsx('dot', { activeDot: i === current })} onClick={() => setCurrent(i)} />
          ))}
        </div>
      </div>
    </section>
  );
}

// --- MAIN HOME COMPONENT ---
export default function Home() {
  return (
    <Layout title="Physical AI & Humanoid Robotics">
      <HomepageHero />
      <ValuePropsSection />
      <TimelineSection />
      <main>
        <ChapterOverviewSection />
        <WhatMakesThisBookDifferentSection />
        <TestimonialsSection />
      </main>
    </Layout>
  );
}