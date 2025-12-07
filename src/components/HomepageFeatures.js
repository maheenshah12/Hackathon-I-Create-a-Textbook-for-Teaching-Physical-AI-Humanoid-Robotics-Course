import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI Integration',
    Svg: require('../../static/img/ai-icon.svg').default,
    description: (
      <>
        Learn how to bridge AI models with physical robots, specifically humanoids.
        Understand the integration between perception, planning, and action.
      </>
    ),
  },
  {
    title: 'ROS2 Foundations',
    Svg: require('../../static/img/ros-icon.svg').default,
    description: (
      <>
        Master the fundamentals of Robot Operating System 2 for controlling humanoid robots.
        Learn about nodes, topics, services, and actions.
      </>
    ),
  },
  {
    title: 'Simulation & Control',
    Svg: require('../../static/img/simulation-icon.svg').default,
    description: (
      <>
        Work with digital twins using Gazebo and Unity for safe testing.
        Implement advanced control algorithms for humanoid locomotion.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} alt={title} />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
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