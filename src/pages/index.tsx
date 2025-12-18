import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className={styles.heroTitle}>
              {siteConfig.title}
            </Heading>
            <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Start Learning
              </Link>
              <Link
                className="button button--outline button--lg"
                to="/docs/modules/ros2-fundamentals/intro">
                Explore ROS 2
              </Link>
            </div>
            <div className={styles.heroStats}>
              <div className={styles.stat}>
                <span className={styles.statNumber}>4+</span>
                <span className={styles.statLabel}>Core Modules</span>
              </div>
              <div className={styles.stat}>
                <span className={styles.statNumber}>20+</span>
                <span className={styles.statLabel}>Hands-on Labs</span>
              </div>
              <div className={styles.stat}>
                <span className={styles.statNumber}>100%</span>
                <span className={styles.statLabel}>Practical Focus</span>
              </div>
            </div>
          </div>
          <div className={styles.heroImage}>
            <img
              src="/img/robot-logo.svg"
              alt="Physical AI Robot"
              className={styles.robotImage}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

function CourseHighlights(): ReactNode {
  return (
    <section className={styles.highlights}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          Why Choose This Course?
        </Heading>
        <div className={styles.highlightGrid}>
          <div className={styles.highlightCard}>
            <div className={styles.highlightIcon}>🎯</div>
            <h3>Industry-Ready Skills</h3>
            <p>Learn the exact technologies used by leading robotics companies worldwide.</p>
          </div>
          <div className={styles.highlightCard}>
            <div className={styles.highlightIcon}>🔬</div>
            <h3>Research-Backed</h3>
            <p>Content based on latest research in embodied AI and humanoid robotics.</p>
          </div>
          <div className={styles.highlightCard}>
            <div className={styles.highlightIcon}>💻</div>
            <h3>Hands-On Projects</h3>
            <p>Build real robotic systems with comprehensive lab exercises and capstone.</p>
          </div>
          <div className={styles.highlightCard}>
            <div className={styles.highlightIcon}>🚀</div>
            <h3>Future-Proof</h3>
            <p>Stay ahead with cutting-edge VLA models and NVIDIA Isaac integration.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Course"
      description="Comprehensive course on ROS 2, Simulation, NVIDIA Isaac, and Vision-Language-Action Models for building intelligent humanoid robots">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <CourseHighlights />
      </main>
    </Layout>
  );
}
