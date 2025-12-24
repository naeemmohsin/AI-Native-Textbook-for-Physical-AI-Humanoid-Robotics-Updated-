import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string;
  description: ReactNode;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 Fundamentals',
    icon: '🤖',
    description: (
      <>
        Master the Robot Operating System 2 (ROS 2) - the foundation for modern
        robotics development. Learn nodes, topics, services, and actions.
      </>
    ),
    link: '/docs/module-1-ros2',
  },
  {
    title: 'NVIDIA Isaac',
    icon: '🧠',
    description: (
      <>
        Build AI-powered robot brains with NVIDIA Isaac. Explore simulation,
        perception, and hardware acceleration for physical AI systems.
      </>
    ),
    link: '/docs/module-3-nvidia-isaac',
  },
  {
    title: 'Vision-Language-Action',
    icon: '👁️',
    description: (
      <>
        Integrate multimodal AI for robot control. Learn how Vision-Language-Action
        (VLA) models enable robots to understand and act on natural language.
      </>
    ),
    link: '/docs/module-4-vla',
  },
];

function Feature({title, icon, description, link}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <span className={styles.featureIcon} role="img" aria-label={title}>
          {icon}
        </span>
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">
          <Link to={link} className={styles.featureLink}>
            {title}
          </Link>
        </Heading>
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
