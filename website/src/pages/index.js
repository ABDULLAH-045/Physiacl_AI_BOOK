import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';


import Heading from '@theme/Heading';
import styles from './index.module.css';

function IntroPageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">
          Explore the world of Physical AI and Humanoid Robotics. From foundational concepts to advanced applications, this book guides you through building intelligent systems that interact with the physical world.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/modules/module1-ros2/chapter0-introduction/">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function IntroPage() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Humanoid Robotics Book"
      description="Description will go into a meta tag in <head />">
      <IntroPageHeader />
      <main>

      </main>
    </Layout>
  );
}
