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
      link: '/docs/01-intro/',
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
  const heroImg = useBaseUrl('/img/book-cover.png');
  return (
    <header className={clsx('hero hero--primary', 'heroBanner')}>  
      <div className="container" style={{maxWidth: 850, margin: '0 auto', padding: '0 1.2rem'}}>
        <div className="hero__inner" style={{display:'flex',flexDirection:'column',alignItems:'center',justifyContent:'center'}}>
          <img 
            src={heroImg}
            alt="Physical AI & Humanoid Robotics book cover" 
            style={{width:'100%',maxWidth:320,objectFit:'contain',borderRadius:18,marginBottom:'2.2rem',boxShadow:'0 7px 32px rgba(46, 133, 85, 0.08)'}} 
          />
          <Heading as="h1" className="hero__title" style={{textAlign:'center'}}>Physical AI & Humanoid Robotics</Heading>
          <div className="hero__badges" style={{display:'flex', flexWrap:'wrap', justifyContent:'center', gap:'.7rem', margin:'1.5rem 0 .6rem 0'}}>
            <span className="heroBadge heroBadge--pill">Spec-Driven</span>
            <span className="heroBadge">AI-Native</span>
            <span className="heroBadge">Graduate Capstone</span>
            <span className="heroBadge">Simulation-First</span>
            <span className="heroBadge">Real Robots</span>
          </div>
          <p className="hero__subtitle" style={{fontSize:'1.22rem',textAlign:'center',maxWidth:520}}>
            <span className="textHighlight">Design, simulate, and deploy</span> a full-stack humanoid—bridging <span className="textHighlight">ROS 2</span>, digital twins, vision-language-action, and ethical autonomy. Built on living specs and labs tested by the world’s top research and industry stacks.
          </p>
          <div className="buttons" style={{justifyContent: 'center', marginTop: '2rem'}}>
            <Link className="button button--secondary button--lg" to="/docs/01-intro/">View Full Course Spec</Link>
            <Link className="button button--info button--lg" to="#book-journey">Book Journey & Topics</Link>
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
    <section style={{background:'var(--ifm-background-surface)',padding:'3rem 0 2.4rem 0'}}>
      <div className="container" style={{maxWidth: 1080, margin:'0 auto'}}>
        <div style={{display:'flex',flexWrap:'wrap',justifyContent:'center',gap:'2.4rem'}}>
        {props.map((p,idx)=>(
          <div key={idx} style={{flex:'1 1 212px',maxWidth:320,minWidth:190,background:'#fff',borderRadius:18,boxShadow:'0 3px 18px rgba(46,133,85,0.08)',display:'flex',flexDirection:'column',alignItems:'center',padding:'2rem 1.3rem'}}>
            <img 
              src={resolve(p.icon)} 
              alt={p.headline+" icon"} 
              style={{width:72,height:72,objectFit:'contain',marginBottom:'1.2rem'}} 
            />
            <h3 style={{fontWeight:700,color:'var(--ifm-color-primary)',fontSize:'1.08rem',marginBottom:'.7rem',marginTop:0,textAlign:'center'}}>{p.headline}</h3>
            <div style={{fontSize:'.97rem',color:'var(--ifm-font-color-secondary)',textAlign:'center'}}>{p.text}</div>
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
    <section id="book-journey" style={{padding:'4.5rem 0 3.5rem 0'}}>
      <div className="container" style={{maxWidth: 960, margin:'0 auto'}}>
        <Heading as="h2" className="text--center" style={{marginBottom:'2.2rem'}}>Your Journey At a Glance</Heading>
        <div style={{display:'flex',flexWrap:'wrap',justifyContent:'center',gap:'1.5rem'}}>
          {timeline.map((step,i)=>
            <div key={i} style={{minWidth:170,maxWidth:240,background:'#fff',borderRadius:14,boxShadow:'0 2px 11px rgba(46,133,85,0.09)',padding:'1.6rem 1.1rem',textAlign:'center',display:'flex',flexDirection:'column'}}>
              <h3 style={{fontWeight:700,fontSize:'1.04rem',margin:0,color:'var(--ifm-color-primary)'}}>{step.title}</h3>
              <p style={{fontSize:'0.96rem',lineHeight:'1.6',margin:'.7rem 0 0 0',color:'var(--ifm-font-color-secondary)'}}>{step.desc}</p>
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
  const filteredChapters = activeFilter==='All' ? chapters : chapters.filter(ch=>ch.tags.includes(activeFilter));
  return (
    <section className="chapterOverviewSection" style={{padding:'4.2rem 0 3.7rem 0',background:'var(--ifm-background-color)'}}>
      <div className="container" style={{maxWidth:1160,margin:'0 auto'}}>
        <Heading as="h2" className="text--center" style={{marginBottom:'2.3rem'}}>Explore All Chapters & Topics</Heading>
        <div className="chapterFilters" style={{marginBottom:'2.7rem',display:'flex',flexWrap:'wrap',justifyContent:'center',gap:'0.7rem'}}>
          {chapterFilters.map(filter => (
            <button key={filter.id} className={clsx('chapterFilterChip',activeFilter===filter.id&&'chapterFilterChip--active')} type="button" onClick={()=>setActiveFilter(filter.id)}>{filter.label}</button>
          ))}
        </div>
        <div style={{display:'grid',gridTemplateColumns:'repeat(auto-fit,minmax(300px,1fr))',gap:'2rem',justifyContent:'center',marginTop:'1.2rem'}}>
        {filteredChapters.map((chapter,idx)=>(
          <Link to={chapter.link} className="chapterCard" key={chapter.id} style={{display:'flex',flexDirection:'column',background:'#fff',borderRadius:18,boxShadow:'0 2px 15px rgba(46,133,85,0.08)',padding:'2.1rem 1.7rem',textDecoration:'none',color:'inherit',height:'100%',minHeight:340}}>
            <div style={{display:'flex',alignItems:'center',gap:'1.1rem',marginBottom:'1.2rem'}}>
              <span className="chapterOrder" style={{fontSize:'.95rem',fontWeight:600,textTransform:'uppercase',letterSpacing:'.10em',color:'var(--ifm-color-emphasis-600)'}}>{chapter.order}</span>
              <img src={useBaseUrl(chapter.icon)} alt={chapter.title+" icon"} className="chapterIcon" style={{width:48,height:48,objectFit:'contain'}}/>
            </div>
            <h3 style={{fontWeight:700,fontSize:'1.16rem',color:'var(--ifm-color-primary)',margin:'0 0 .7rem 0'}}>{chapter.title}</h3>
            <p style={{fontSize:'0.97rem',lineHeight:'1.7',color:'var(--ifm-font-color-secondary)',marginTop:0}}>{chapter.description}</p>
            <div style={{display:'flex',gap:'.33rem',flexWrap:'wrap',marginTop:'1.3rem'}}>
              {chapter.highlights.map(item=>(<span key={item} className="chapterHighlightPill">{item}</span>))}
            </div>
            <div style={{marginTop:'.6rem',display:'flex',flexWrap:'wrap',gap:'.35rem'}}>
              {chapter.tags.map(tag=>(<span key={tag} className="chapterTagPill">{tag}</span>))}
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
    {icon:'/img/undraw_docusaurus_tree.svg',text:'Spec-driven, AI-powered workflow where docs are code, and labs always match your real robot progress.'},
    {icon:'/img/undraw_docusaurus_mountain.svg',text:'Everything proven in simulation before real hardware. Safe, industry-standard stack.'},
    {icon:'/img/undraw_docusaurus_react.svg',text:'Language models, perception, ROS 2: you build a complete, world-ready stack.'},
    {icon:'/img/undraw_docusaurus_tree.svg',text:'Learning-enabled—Reinforcement/Imitation directly connect to simulation and perception.'},
    {icon:'/img/undraw_docusaurus_mountain.svg',text:'Ethics and lab governance are never an afterthought: trust and safety by design.'},
        {icon:'/img/undraw_docusaurus_react.svg',text:'Humanoids as grand challenge: bipedal walking, hands, vision, language, autonomy.'},
  ];
  const resolve = useBaseUrl;
  return (
    <section className="whatMakesThisBookDifferentSection" style={{padding:'4.2rem 0 3.7rem 0'}}>
      <div className="container" style={{maxWidth:940,margin:'0 auto'}}>
        <Heading as="h2" className="text--center" style={{marginBottom:'2.1rem'}}>What Makes This Book Different?</Heading>
        <div className="featureCardsGrid" style={{display:'grid',gridTemplateColumns:'repeat(auto-fit,minmax(260px,1fr))',gap:'2.1rem',justifyContent:'center'}}>
        {features.map((card,idx)=>(
          <div key={idx} className="featureCard" style={{background:'#fff',borderRadius:15,boxShadow:'0 2px 10px rgba(46,133,85,0.09)',display:'flex',flexDirection:'column',alignItems:'center',padding:'2.2rem 1.2rem',minHeight:210}}>
            <img src={resolve(card.icon)} alt="Feature icon" style={{width:60,height:60,objectFit:'contain',marginBottom:'1.2rem'}}/>
            <p style={{fontSize:'1.03rem',color:'var(--ifm-font-color-secondary)',textAlign:'center',margin:0}}>{card.text}</p>
          </div>
        ))}
        </div>
      </div>
    </section>
  );
}

// --- TESTIMONIALS SECTION ---
const testimonials = [
  {quote:"An essential read for anyone interested in the future of AI. The concepts are explained with remarkable clarity!",author:"- Dr. Alex Tech, AI Researcher"},
  {quote:"This book transformed my understanding of embodied AI. Highly recommended for both beginners and experts.",author:"- Sarah Innovate, Robotics Engineer"},
  {quote:"Finally, a comprehensive guide to Physical AI that is both insightful and practical. A must-have!",author:"- Prof. J. Data, Computer Science Dept."},
  {quote:"A groundbreaking work that bridges the gap between theoretical AI and its real-world physical applications.",author:"- Dr. Emily Robotica, Lead AI Scientist"},
];
function TestimonialsSection() {
  const [current, setCurrent] = useState(0);
  useEffect(()=>{ const timer = setInterval(()=>setCurrent((p)=>(p+1)%testimonials.length), 6500); return ()=>clearInterval(timer); },[]);
  return (
    <section className="testimonialsSection" style={{padding:'4rem 0 3rem 0',background:'var(--ifm-background-color)'}}>
      <div className="container" style={{maxWidth:700,margin:'0 auto'}}>
        <Heading as="h2" className="text--center" style={{marginBottom:'2rem'}}>What Readers Say</Heading>
        <div className="testimonialSlider" style={{minHeight:220,position:'relative',background:'#fff',borderRadius:14,boxShadow:'0 2px 10px rgba(46,133,85,0.06)',padding:'2.4rem'}}>
          {testimonials.map((t,i)=>(
            <div key={i} className={clsx('testimonialItem', {activeTestimonial:i===current})} style={{opacity:i===current?1:0,position:i===current?'relative':'absolute',transition:'opacity 0.5s',transitionDelay:i===current?'.15s':'0s'}}>
              <p className="testimonialQuote">"{t.quote}"</p>
              <p className="testimonialAuthor">{t.author}</p>
            </div>
          ))}
        </div>
        <div className="testimonialDots" style={{marginTop:'1.7rem',justifyContent:'center',display:'flex',gap:'.8rem'}}>
          {testimonials.map((_,i)=>(
            <span key={i} className={clsx('dot',{activeDot:i===current})} onClick={()=>setCurrent(i)} />
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