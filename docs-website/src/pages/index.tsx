import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useBaseUrlUtils } from '@docusaurus/useBaseUrl';

export default function Home() {
  const { withBaseUrl } = useBaseUrlUtils();

  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="An AI-native, interactive textbook teaching embodied intelligence with real robots and simulations."
    >
      <main>
        <div style={{ paddingTop: '2rem', paddingBottom: '2rem' }}>
          <div style={{ maxWidth: '1200px', margin: '0 auto', padding: '0 2rem' }}>
            <h1 style={{ fontSize: '3rem', fontWeight: '700', marginBottom: '1rem', textAlign: 'center' }}>
              Physical AI & Humanoid Robotics
            </h1>
            <p style={{ fontSize: '1.25rem', textAlign: 'center', color: '#666', marginBottom: '3rem' }}>
              From Code to Corpus: The Guide to Embodied Intelligence.
            </p>

            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))', gap: '2rem', marginBottom: '3rem' }}>
              {/* Module Cards */}
              <ModuleCard
                title="Module 1: The Nervous System"
                description="Foundation of Robot Communication & Control with ROS 2"
                link="/01-nervous-system"
              />
              <ModuleCard
                title="Module 2: Digital Twin"
                description="Building Virtual Replicas with Isaac Sim"
                link="/02-digital-twin"
              />
              <ModuleCard
                title="Module 3: Robot Brain"
                description="Perception & Planning with VLMs"
                link="/03-robot-brain"
              />
              <ModuleCard
                title="Module 4: The Mind"
                description="Vision Language Models & Advanced Reasoning"
                link="/04-the-mind"
              />
              <ModuleCard
                title="Module 5: Capstone Project"
                description="End-to-End Autonomous System on Real Hardware"
                link="/05-capstone"
              />
            </div>

            {/* Features Section */}
            <div style={{ backgroundColor: '#f3f4f6', padding: '3rem 2rem', borderRadius: '8px', marginBottom: '3rem' }}>
              <h2 style={{ textAlign: 'center', marginBottom: '2rem' }}>Key Features</h2>
              <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))', gap: '2rem' }}>
                <Feature icon="📚" title="5 Curriculum Modules" description="ROS 2 → Digital Twin → Isaac Sim → VLA → Capstone" />
                <Feature icon="🤖" title="RAG Chatbot" description="'Ask the Book' widget for context-aware Q&A" />
                <Feature icon="🔐" title="Authentication" description="User signup with hardware/software profiles" />
                <Feature icon="🎯" title="Personalization" description="Content adapted to your background" />
                <Feature icon="🌍" title="Localization" description="English + Urdu with RTL support" />
                <Feature icon="⚡" title="CI/CD Pipeline" description="Automated deployment to GitHub Pages" />
              </div>
            </div>

            {/* Hardware Requirements */}
            <div style={{ marginBottom: '3rem' }}>
              <h2 style={{ textAlign: 'center', marginBottom: '2rem' }}>Hardware Requirements</h2>
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
                to="/01-nervous-system"
              >
                Start Module 1: The Nervous System →
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
        <h3 style={{ marginTop: 0 }}>{title}</h3>
        <p style={{ color: '#666', flexGrow: 1 }}>{description}</p>
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
