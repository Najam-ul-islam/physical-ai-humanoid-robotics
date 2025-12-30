import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Humanoid Robotics',
    description: (
      <>
        Explore the cutting-edge world of humanoid robots, from their mechanical design to
        advanced control systems that enable human-like movement and interaction.
      </>
    ),
  },
  {
    title: 'Physical AI',
    description: (
      <>
        Understand how artificial intelligence is integrated with physical systems to create
        intelligent robots that can perceive, learn, and adapt to their physical environment.
      </>
    ),
  },
  {
    title: 'Physics Simulation',
    description: (
      <>
        Learn about the complex physics simulations that enable robots to understand and
        interact with the physical world through accurate modeling and real-time computation.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.featureBox}>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
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
