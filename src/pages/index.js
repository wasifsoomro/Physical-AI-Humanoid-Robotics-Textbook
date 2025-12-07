import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started with the Book - 5 min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Educational Book">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4 padding-horiz--md">
                <h2>Module 1: The Robotic Nervous System (ROS 2)</h2>
                <p>Learn the fundamentals of ROS 2, including Nodes, Topics, Services, and Actions.</p>
                <Link to="/docs/module1/chapter1">Start Module 1 →</Link>
              </div>
              <div className="col col--4 padding-horiz--md">
                <h2>Module 2: Digital Twin Simulation (Gazebo + Unity)</h2>
                <p>Discover how to create and simulate humanoid robots in realistic environments using Gazebo.</p>
                <Link to="/docs/module2/chapter1">Start Module 2 →</Link>
              </div>
              <div className="col col--4 padding-horiz--md">
                <h2>Module 3: AI-Robot Brain (NVIDIA Isaac + Isaac ROS)</h2>
                <p>Explore perception, mapping, navigation, and synthetic data generation for intelligent robots.</p>
                <Link to="/docs/module3/chapter1">Start Module 3 →</Link>
              </div>
            </div>
            <div className="row" style={{marginTop: "2rem"}}>
              <div className="col col--4 padding-horiz--md">
                <h2>Module 4: Vision-Language-Action Robotics (VLA)</h2>
                <p>Master how robots understand natural language commands and execute complex physical actions.</p>
                <Link to="/docs/module4/chapter1">Start Module 4 →</Link>
              </div>
              <div className="col col--8 padding-horiz--md">
                <h2>About This Book</h2>
                <p>Welcome to the Physical AI & Humanoid Robotics Educational Book! This comprehensive guide covers the essential concepts needed to understand and implement humanoid robotics systems using modern AI and simulation technologies.</p>
                <p>This book is designed for students in grades 5-8 with an interest in robotics, AI, and computer science. It provides hands-on experience with ROS 2, Gazebo, and NVIDIA Isaac Sim.</p>
                <p>All code examples are in Python and are designed to be reproducible, making it easy to follow along and build your own understanding through practice.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}