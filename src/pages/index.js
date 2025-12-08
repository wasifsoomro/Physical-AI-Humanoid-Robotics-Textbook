import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="row">
          <div className="col col--6">
            <h1 className="hero__title">{siteConfig.title}</h1>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <p className={styles.heroDescription}>
              Master the cutting-edge field of humanoid robotics with this comprehensive educational resource.
              Learn to build, simulate, and control intelligent humanoid robots using the latest technologies.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Start Learning Now
              </Link>
              <Link
                className="button button--outline button--primary button--lg margin-left--md"
                to="/docs/module1/chapter1">
                Explore Modules
              </Link>
            </div>
          </div>
          <div className="col col--6">
            <div className={styles.heroImageContainer}>
              <div className={styles.heroImage}>
                <svg className={styles.robotIcon} viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg">
                  <circle cx="50" cy="30" r="12" fill="white" opacity="0.9"/>
                  <rect x="40" y="42" width="20" height="30" fill="white" opacity="0.9"/>
                  <rect x="30" y="45" width="10" height="15" fill="white" opacity="0.9"/>
                  <rect x="60" y="45" width="10" height="15" fill="white" opacity="0.9"/>
                  <rect x="35" y="72" width="8" height="20" fill="white" opacity="0.9"/>
                  <rect x="57" y="72" width="8" height="20" fill="white" opacity="0.9"/>
                </svg>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({ title, description, icon, link, linkText }) {
  return (
    <div className="col col--3">
      <div className={styles.featureCard}>
        <div className={styles.featureIcon}>
          {icon}
        </div>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
        {link && linkText && (
          <Link to={link} className={styles.featureLink}>
            {linkText} →
          </Link>
        )}
      </div>
    </div>
  );
}

function ModuleCard({ title, description, link, color }) {
  return (
    <div className="col col--3">
      <div className={clsx(styles.moduleCard, styles[color])}>
        <Heading as="h3" className={styles.moduleTitle}>{title}</Heading>
        <p className={styles.moduleDescription}>{description}</p>
        <Link to={link} className={styles.moduleLink}>
          Start Learning →
        </Link>
      </div>
    </div>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();

  const features = [
    {
      title: 'Comprehensive Curriculum',
      description: 'Covers all aspects of humanoid robotics from basic concepts to advanced AI integration.',
      icon: (
        <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor">
          <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M22 4 12 14.01l-3-3" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      ),
      link: '/docs/intro',
      linkText: 'Get Started'
    },
    {
      title: 'Hands-on Learning',
      description: 'Practical exercises and projects to reinforce theoretical concepts.',
      icon: (
        <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor">
          <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <polyline points="14,2 14,8 20,8" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M16 13H8" strokeWidth="2" strokeLinecap="round"/>
          <path d="M16 17H8" strokeWidth="2" strokeLinecap="round"/>
          <path d="M10 9H9L8 9" strokeWidth="2" strokeLinecap="round"/>
        </svg>
      ),
      link: '/docs/module1/chapter1',
      linkText: 'Start Coding'
    },
    {
      title: 'Modern Technologies',
      description: 'Learn with industry-standard tools like ROS 2, Gazebo, NVIDIA Isaac, and more.',
      icon: (
        <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor">
          <rect x="2" y="3" width="20" height="14" rx="2" strokeWidth="2"/>
          <line x1="8" y1="21" x2="16" y2="21" strokeWidth="2" strokeLinecap="round"/>
          <line x1="12" y1="17" x2="12" y2="21" strokeWidth="2" strokeLinecap="round"/>
        </svg>
      ),
      link: '/docs/module1/chapter1',
      linkText: 'Explore Tech'
    },
    {
      title: 'Educational Focus',
      description: 'Designed specifically for students with clear explanations and structured learning paths.',
      icon: (
        <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor">
          <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <polyline points="14,2 14,8 20,8" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <line x1="16" y1="13" x2="8" y2="13" strokeWidth="2" strokeLinecap="round"/>
          <line x1="16" y1="17" x2="8" y2="17" strokeWidth="2" strokeLinecap="round"/>
          <polyline points="10,9 9,9 8,9" strokeWidth="2" strokeLinecap="round"/>
        </svg>
      ),
      link: '/docs/intro',
      linkText: 'Learn More'
    }
  ];

  const modules = [
    {
      title: 'Module 1: Robotic Nervous System (ROS 2)',
      description: 'Learn the fundamentals of ROS 2, including Nodes, Topics, Services, and Actions.',
      link: '/docs/module1/chapter1',
      color: 'module1'
    },
    {
      title: 'Module 2: Digital Twin Simulation (Gazebo)',
      description: 'Create and simulate humanoid robots in realistic environments using Gazebo.',
      link: '/docs/module2/chapter1',
      color: 'module2'
    },
    {
      title: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
      description: 'Explore perception, mapping, navigation, and synthetic data generation.',
      link: '/docs/module3/chapter1',
      color: 'module3'
    },
    {
      title: 'Module 4: Vision-Language-Action Robotics',
      description: 'Master how robots understand natural language and execute complex actions.',
      link: '/docs/module4/chapter1',
      color: 'module4'
    }
  ];

  return (
    <Layout
      title={`Physical AI & Humanoid Robotics Book`}
      description="Comprehensive educational resource for humanoid robotics with ROS 2, Gazebo, and NVIDIA Isaac">
      <HomepageHeader />
      <main>
        {/* Features Section */}
        <section className={styles.featuresSection}>
          <div className="container padding-vert--lg">
            <div className="row">
              {features.map((feature, index) => (
                <FeatureCard
                  key={index}
                  title={feature.title}
                  description={feature.description}
                  icon={feature.icon}
                  link={feature.link}
                  linkText={feature.linkText}
                />
              ))}
            </div>
          </div>
        </section>

        {/* Modules Section */}
        <section className={styles.modulesSection}>
          <div className="container padding-vert--lg">
            <div className="row">
              <div className="col col--12">
                <Heading as="h2" className={styles.sectionTitle}>
                  Learning Modules
                </Heading>
                <p className={styles.sectionSubtitle}>
                  Structured curriculum designed to take you from beginner to advanced in humanoid robotics
                </p>
              </div>
            </div>
            <div className="row">
              {modules.map((module, index) => (
                <ModuleCard
                  key={index}
                  title={module.title}
                  description={module.description}
                  link={module.link}
                  color={module.color}
                />
              ))}
            </div>
          </div>
        </section>

        {/* About Section */}
        <section className={styles.aboutSection}>
          <div className="container padding-vert--lg">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <Heading as="h2" className={styles.sectionTitle}>
                  About This Book
                </Heading>
                <p className={styles.aboutText}>
                  Welcome to the Physical AI & Humanoid Robotics Educational Book! This comprehensive guide covers the essential concepts needed to understand and implement humanoid robotics systems using modern AI and simulation technologies.
                </p>
                <p className={styles.aboutText}>
                  This book is designed for students in grades 5-8 with an interest in robotics, AI, and computer science. It provides hands-on experience with ROS 2, Gazebo, and NVIDIA Isaac Sim.
                </p>
                <p className={styles.aboutText}>
                  All code examples are in Python and are designed to be reproducible, making it easy to follow along and build your own understanding through practice.
                </p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}