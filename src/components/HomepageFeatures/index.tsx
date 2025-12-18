import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 Fundamentals',
    Svg: require('@site/static/img/ros2-feature.svg').default,
    description: (
      <>
        Master Robot Operating System 2 (ROS 2) with hands-on tutorials covering
        nodes, topics, services, actions, and real-time communication for modern robotics.
      </>
    ),
  },
  {
    title: 'Advanced Simulation',
    Svg: require('@site/static/img/simulation-feature.svg').default,
    description: (
      <>
        Build and test robots in high-fidelity simulation environments using
        Gazebo and Unity. Iterate faster with physics-accurate virtual testing.
      </>
    ),
  },
  {
    title: 'NVIDIA Isaac Platform',
    Svg: require('@site/static/img/nvidia-isaac-feature.svg').default,
    description: (
      <>
        Leverage GPU-accelerated simulation with NVIDIA Isaac Sim. Train AI models
        and deploy to real hardware with seamless sim-to-real transfer.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action Models',
    Svg: require('@site/static/img/vla-feature.svg').default,
    description: (
      <>
        Implement cutting-edge VLA models that combine computer vision, natural
        language understanding, and robotic control for intelligent humanoid behaviors.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--6', styles.featureCol)}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
