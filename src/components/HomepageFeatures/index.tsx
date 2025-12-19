import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
  tags?: string[];
  link: string;
  chapters: {title: string; link: string}[];
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 Fundamentals',
    Svg: require('@site/static/img/ros2-feature.svg').default,
    description: (
      <>
        Master Robot Operating System 2 (ROS 2) with hands-on tutorials covering
        nodes, topics, services, actions, and real-time communication.
      </>
    ),
    tags: ['Nodes', 'Topics', 'Services'],
    link: '/docs/modules/ros2-fundamentals/intro',
    chapters: [
      {title: 'Introduction to ROS 2', link: '/docs/modules/ros2-fundamentals/chapter-1-introduction'},
      {title: 'Nodes, Topics & Services', link: '/docs/modules/ros2-fundamentals/chapter-2-nodes-topics-services'},
      {title: 'Custom Messages', link: '/docs/modules/ros2-fundamentals/chapter-3-custom-messages'},
      {title: 'Tools & Debugging', link: '/docs/modules/ros2-fundamentals/chapter-4-tools-debugging'},
    ],
  },
  {
    title: 'Advanced Simulation',
    Svg: require('@site/static/img/simulation-feature.svg').default,
    description: (
      <>
        Build and test robots in high-fidelity simulation environments using
        Gazebo and Unity with physics-accurate virtual testing.
      </>
    ),
    tags: ['Gazebo', 'Unity', 'Physics'],
    link: '/docs/modules/simulation/intro',
    chapters: [
      {title: 'Simulation Overview', link: '/docs/modules/simulation/intro'},
    ],
  },
  {
    title: 'NVIDIA Isaac Platform',
    Svg: require('@site/static/img/nvidia-isaac-feature.svg').default,
    description: (
      <>
        Leverage GPU-accelerated simulation with NVIDIA Isaac Sim. Train AI models
        and deploy to real hardware with sim-to-real transfer.
      </>
    ),
    tags: ['GPU', 'Isaac Sim', 'Sim2Real'],
    link: '/docs/modules/nvidia-isaac/intro',
    chapters: [
      {title: 'Isaac Platform Overview', link: '/docs/modules/nvidia-isaac/intro'},
    ],
  },
  {
    title: 'Vision-Language-Action Models',
    Svg: require('@site/static/img/vla-feature.svg').default,
    description: (
      <>
        Implement cutting-edge VLA models that combine vision, language,
        and robotic control for intelligent humanoid behaviors.
      </>
    ),
    tags: ['VLA', 'AI', 'Humanoid'],
    link: '/docs/modules/vla-integration/intro',
    chapters: [
      {title: 'VLA Models Overview', link: '/docs/modules/vla-integration/intro'},
    ],
  },
];

function Feature({title, Svg, description, tags, link, chapters, index}: FeatureItem & {index: number}) {
  return (
    <div className={styles.featureCard}>
      <Link to={link} className={styles.featureCardLink}>
        <div className={styles.featureImageContainer}>
          <span className={styles.featureNumber}>{index + 1}</span>
          <Svg className={styles.featureSvg} role="img" />
        </div>
        <div className={styles.featureContent}>
          <h3 className={styles.featureTitle}>{title}</h3>
          <p className={styles.featureDescription}>{description}</p>
          {tags && tags.length > 0 && (
            <div className={styles.featureTags}>
              {tags.map((tag, idx) => (
                <span key={idx} className={styles.featureTag}>{tag}</span>
              ))}
            </div>
          )}
        </div>
        <div className={styles.featureArrow}>
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M5 12h14M12 5l7 7-7 7"/>
          </svg>
        </div>
      </Link>

      {/* Chapters dropdown for desktop */}
      {chapters.length > 0 && (
        <div className={styles.chaptersDropdown}>
          <div className={styles.chaptersHeader}>
            <span className={styles.chaptersIcon}>
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20"/>
                <path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z"/>
              </svg>
            </span>
            <span>{chapters.length} Chapter{chapters.length > 1 ? 's' : ''}</span>
          </div>
          <ul className={styles.chaptersList}>
            {chapters.map((chapter, idx) => (
              <li key={idx}>
                <Link to={chapter.link} className={styles.chapterLink}>
                  <span className={styles.chapterNumber}>{idx + 1}</span>
                  <span className={styles.chapterTitle}>{chapter.title}</span>
                </Link>
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Course Modules
          </Heading>
          <p className={styles.sectionSubtitle}>
            Click on any module to explore chapters and start learning
          </p>
        </div>
        <div className={styles.featureGrid}>
          {FeatureList.map((props, idx) => (
            <Feature key={idx} index={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
