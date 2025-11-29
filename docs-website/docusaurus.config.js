// @ts-check

const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'An AI-native, interactive textbook teaching embodied intelligence with real robots and simulations.',
  favicon: 'img/favicon.ico',

  url: 'https://username.github.io',
  baseUrl: '/BookProject/',
  organizationName: 'username',
  projectName: 'BookProject',
  deploymentBranch: 'gh-pages',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/username/BookProject/tree/main/docs-website/',
          routeBasePath: '/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI Textbook',
      logo: {
        alt: 'Site Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'localeDropdown',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Curriculum',
          items: [
            { label: 'Module 1: The Nervous System', to: '/01-nervous-system' },
            { label: 'Module 2: Digital Twin', to: '/02-digital-twin' },
            { label: 'Module 3: Robot Brain', to: '/03-robot-brain' },
            { label: 'Module 4: The Mind', to: '/04-the-mind' },
            { label: 'Module 5: Capstone', to: '/05-capstone' },
          ],
        },
        {
          title: 'Resources',
          items: [
            { label: 'GitHub', href: 'https://github.com/username/BookProject' },
            { label: 'Specification', href: 'https://github.com/username/BookProject/blob/main/specs/001-ai-textbook/spec.md' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built with Docusaurus.`,
    },
  },
};

module.exports = config;
