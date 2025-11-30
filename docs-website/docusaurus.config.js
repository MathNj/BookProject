// @ts-check

const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'An AI-native, interactive textbook teaching embodied intelligence with real robots and simulations.',
  favicon: 'img/favicon.ico',

  url: 'https://mathnj.github.io',
  baseUrl: '/BookProject/',
  organizationName: 'MathNj',
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
          editUrl: 'https://github.com/MathNj/BookProject/tree/main/docs-website/',
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
            { label: 'Module 1: The Nervous System', to: '/nervous-system/getting-started' },
            { label: 'Module 2: Digital Twin', to: '/digital-twin-sim/getting-started' },
            { label: 'Module 3: Robot Brain', to: '/robot-brain/intro' },
            { label: 'Module 4: The Mind', to: '/the-mind/intro' },
            { label: 'Module 5: Capstone', to: '/capstone/intro' },
          ],
        },
        {
          title: 'Resources',
          items: [
            { label: 'GitHub', href: 'https://github.com/MathNj/BookProject' },
            { label: 'Specification', href: 'https://github.com/MathNj/BookProject/blob/main/specs/001-ai-textbook/spec.md' },
            { label: 'NVIDIA Isaac Sim', href: 'https://docs.omniverse.nvidia.com/isaacsim/latest/' },
            { label: 'ROS 2 Documentation', href: 'https://docs.ros.org/en/humble/' },
          ],
        },
      ],
      copyright: `© 2024 Physical AI Initiative. Built with Spec-Kit Plus.`,
    },
  },
};

module.exports = config;
