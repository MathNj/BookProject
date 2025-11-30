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
        link: '/01-nervous-system',
      },
      {
        title: 'Module 2: Digital Twin',
        description: 'Building Virtual Replicas with Isaac Sim',
        link: '/02-digital-twin',
      },
      {
        title: 'Module 3: Robot Brain',
        description: 'Perception & Planning with VLMs',
        link: '/03-robot-brain',
      },
      {
        title: 'Module 4: The Mind',
        description: 'Vision Language Models & Advanced Reasoning',
        link: '/04-the-mind',
      },
      {
        title: 'Module 5: Capstone Project',
        description: 'End-to-End Autonomous System on Real Hardware',
        link: '/05-capstone',
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
        title: 'Localization',
        description: 'English + Urdu with RTL support',
      },
      {
        icon: '⚡',
        title: 'CI/CD Pipeline',
        description: 'Automated deployment to GitHub Pages',
      },
    ],
  },
  ur: {
    title: 'فزیکل AI اور ہیومانوئڈ روبوٹکس',
    subtitle: 'کوڈ سے کارپس تک: مجسم ذہانت کی رہنما کتاب',
    cta: 'Module 1 شروع کریں: نرووس سسٹم →',
    keyFeatures: 'اہم خصوصیات',
    hardwareRequirements: 'ہارڈویئر کی ضروریات',
    modules: [
      {
        title: 'ماڈیول 1: نرووس سسٹم',
        description: 'ROS 2 کے ساتھ روبوٹ کمیونیکیشن اور کنٹرول کی بنیاد',
        link: '/ur/01-nervous-system',
      },
      {
        title: 'ماڈیول 2: ڈیجیٹل ٹوئن',
        description: 'Isaac Sim کے ساتھ ورچوئل ڈیجیٹل ٹوئن بنائیں',
        link: '/ur/02-digital-twin',
      },
      {
        title: 'ماڈیول 3: روبوٹ کا دماغ',
        description: 'VLMs کے ساتھ ادراک اور منصوبہ بندی',
        link: '/ur/03-robot-brain',
      },
      {
        title: 'ماڈیول 4: ذہن',
        description: 'وژن لینگویج ماڈلز اور جدید استدلال',
        link: '/ur/04-the-mind',
      },
      {
        title: 'ماڈیول 5: اختتامی منصوبہ',
        description: 'حقیقی ہارڈویئر پر مکمل خود مختار نظام',
        link: '/ur/05-capstone',
      },
    ],
    features: [
      {
        icon: '📚',
        title: '5 نصاب ماڈیولز',
        description: 'ROS 2 → ڈیجیٹل ٹوئن → Isaac Sim → VLA → اختتامی منصوبہ',
      },
      {
        icon: '🤖',
        title: 'RAG چیٹ بوٹ',
        description: 'کسی بھی سوال کے جوابات کے لیے \'کتاب سے پوچھیں\' ودجیٹ',
      },
      {
        icon: '🔐',
        title: 'تصدیق',
        description: 'ہارڈویئر اور سافٹویئر پروفائل کے ساتھ صارف کی رجسٹریشن',
      },
      {
        icon: '🎯',
        title: 'ذاتی سازی',
        description: 'آپ کی پس منظر کے مطابق مختلف مواد',
      },
      {
        icon: '🌍',
        title: 'مختلف زبانیں',
        description: 'انگریزی + اردو RTL سپورٹ کے ساتھ',
      },
      {
        icon: '⚡',
        title: 'CI/CD پائپ لائن',
        description: 'GitHub Pages پر خودکار تعینات',
      },
    ],
  },
};

export default function Home() {
  const { withBaseUrl } = useBaseUrlUtils();
  const location = useLocation();

  const isUrdu = useMemo(() => location.pathname.startsWith('/ur/'), [location.pathname]);
  const lang = isUrdu ? 'ur' : 'en';
  const t = translations[lang];

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
                to={isUrdu ? '/ur/01-nervous-system' : '/01-nervous-system'}
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
