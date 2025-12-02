import React, { useMemo } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useBaseUrlUtils } from '@docusaurus/useBaseUrl';
import { useLocation } from '@docusaurus/router';

const translations = {
  en: {
    title: 'Physical AI & Humanoid Robotics',
    subtitle: 'From Code to Corpus: The Guide to Embodied Intelligence.',
    cta: 'Start Module 1: The Nervous System →',
    keyFeatures: 'Key Features',
    hardwareRequirements: 'Hardware Requirements',
    modules: [
      {
        title: 'Module 1: The Nervous System',
        description: 'Foundation of Robot Communication & Control with ROS 2',
        link: '/nervous-system/nervous-system-overview',
      },
      {
        title: 'Module 2: Digital Twin',
        description: 'Building Virtual Replicas with Isaac Sim',
        link: '/digital-twin/digital-twin-overview',
      },
      {
        title: 'Module 3: Robot Brain',
        description: 'Perception & Planning with VLMs',
        link: '/robot-brain/robot-brain-overview',
      },
      {
        title: 'Module 4: The Mind',
        description: 'Vision Language Models & Advanced Reasoning',
        link: '/the-mind/the-mind-overview',
      },
      {
        title: 'Module 5: Capstone Project',
        description: 'End-to-End Autonomous System on Real Hardware',
        link: '/capstone/capstone-overview',
      },
    ],
    features: [
      {
        icon: '📚',
        title: '5 Curriculum Modules',
        description: 'ROS 2 → Digital Twin → Isaac Sim → VLA → Capstone',
      },
      {
        icon: '🤖',
        title: 'RAG Chatbot',
        description: "'Ask the Book' widget for context-aware Q&A",
      },
      {
        icon: '🔐',
        title: 'Authentication',
        description: 'User signup with hardware/software profiles',
      },
      {
        icon: '🎯',
        title: 'Personalization',
        description: 'Content adapted to your background',
      },
      {
        icon: '🌍',
        title: 'Global Access',
        description: 'English with international support',
      },
      {
        icon: '⚡',
        title: 'CI/CD Pipeline',
        description: 'Automated deployment to GitHub Pages',
      },
    ],
  },
};

export default function Home() {
  const { withBaseUrl } = useBaseUrlUtils();

  const t = translations.en;

  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="An AI-native, interactive textbook teaching embodied intelligence with real robots and simulations."
    >
      <main>
        <div style={{ paddingTop: '2rem', paddingBottom: '2rem' }}>
          <div style={{ maxWidth: '1200px', margin: '0 auto', padding: '0 2rem' }}>
            <h1 style={{ fontSize: '3rem', fontWeight: '700', marginBottom: '1rem', textAlign: 'center' }}>
              {t.title}
            </h1>
            <p style={{ fontSize: '1.25rem', textAlign: 'center', color: '#666', marginBottom: '3rem' }}>
              {t.subtitle}
            </p>

            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))', gap: '2rem', marginBottom: '3rem' }}>
              {t.modules.map((module, idx) => (
                <ModuleCard
                  key={idx}
                  title={module.title}
                  description={module.description}
                  link={module.link}
                />
              ))}
            </div>

            {/* Features Section */}
            <div style={{ backgroundColor: '#f3f4f6', padding: '3rem 2rem', borderRadius: '8px', marginBottom: '3rem' }}>
              <h2 style={{ textAlign: 'center', marginBottom: '2rem' }}>{t.keyFeatures}</h2>
              <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))', gap: '2rem' }}>
                {t.features.map((feature, idx) => (
                  <Feature
                    key={idx}
                    icon={feature.icon}
                    title={feature.title}
                    description={feature.description}
                  />
                ))}
              </div>
            </div>

            {/* Hardware Requirements */}
            <div style={{ marginBottom: '3rem' }}>
              <h2 style={{ textAlign: 'center', marginBottom: '2rem' }}>{t.hardwareRequirements}</h2>
              <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))', gap: '2rem' }}>
                <div style={{ border: '1px solid #ddd', padding: '1.5rem', borderRadius: '8px' }}>
                  <h3>Primary Workstation</h3>
                  <ul>
                    <li>GPU: NVIDIA RTX 4070 Ti</li>
                    <li>RAM: 64GB</li>
                    <li>OS: Ubuntu 22.04 LTS</li>
                  </ul>
                </div>
                <div style={{ border: '1px solid #ddd', padding: '1.5rem', borderRadius: '8px' }}>
                  <h3>Edge Device</h3>
                  <ul>
                    <li>NVIDIA Jetson Orin Nano</li>
                    <li>or Jetson Orin NX</li>
                  </ul>
                </div>
                <div style={{ border: '1px solid #ddd', padding: '1.5rem', borderRadius: '8px' }}>
                  <h3>Target Robot</h3>
                  <ul>
                    <li>Unitree Go2 (quadruped)</li>
                    <li>or Unitree G1 (humanoid)</li>
                  </ul>
                </div>
              </div>
            </div>

            {/* CTA */}
            <div style={{ textAlign: 'center' }}>
              <Link
                className="button button--primary button--lg"
                to="/nervous-system/nervous-system-overview"
              >
                {t.cta}
              </Link>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

function ModuleCard({ title, description, link }) {
  return (
    <Link to={link} style={{ textDecoration: 'none', color: 'inherit' }}>
      <div
        style={{
          border: '1px solid #e5e7eb',
          borderRadius: '8px',
          padding: '2rem',
          transition: 'all 0.3s ease',
          cursor: 'pointer',
          height: '100%',
          display: 'flex',
          flexDirection: 'column',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.boxShadow = '0 10px 30px rgba(0,0,0,0.1)';
          e.currentTarget.style.transform = 'translateY(-4px)';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.boxShadow = 'none';
          e.currentTarget.style.transform = 'translateY(0)';
        }}
      >
        <h3 style={{ marginTop: 0 }}>
          {title}
        </h3>
        <p style={{ color: '#666', flexGrow: 1 }}>
          {description}
        </p>
        <span style={{ color: '#3b82f6', fontWeight: '600' }}>Learn more →</span>
      </div>
    </Link>
  );
}

function Feature({ icon, title, description }) {
  return (
    <div style={{ textAlign: 'center' }}>
      <div style={{ fontSize: '2.5rem', marginBottom: '1rem' }}>{icon}</div>
      <h4 style={{ marginBottom: '0.5rem' }}>{title}</h4>
      <p style={{ color: '#666', fontSize: '0.95rem' }}>{description}</p>
    </div>
  );
}
