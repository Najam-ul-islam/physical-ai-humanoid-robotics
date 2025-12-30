import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "Building the Next Generation of Humanoid Robots",
  favicon: "img/favicon.ico",

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: "https://physical-ai-humanoid-robotics-ot6h.vercel.app/",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: "/",

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "Najam-ul-islam", // Usually your GitHub org/user name.
  projectName: "physical-ai-humanoid-robotics", // Usually your repo name.

  onBrokenLinks: "throw",

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/Najam-ul-islam/physical-ai-humanoid-robotics/edit/main/physical-ai-book/",
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ["rss", "atom"],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/Najam-ul-islam/physical-ai-humanoid-robotics/edit/main/physical-ai-book/",
          // Useful options to enforce blogging best practices
          onInlineTags: "warn",
          onInlineAuthors: "warn",
          onUntruncatedBlogPosts: "warn",
        },
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: "img/docusaurus-social-card.jpg",
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: "Physical AI & Humanoid Robotics",
      logo: {
        alt: "Physical AI & Humanoid Robotics Logo",
        src: "img/logo.svg",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "docsSidebar",
          position: "left",
          label: "Browse Book",
        },
        {
          to: "/docs/module-1/ros2-fundamentals",
          label: "Module 1",
          position: "left",
        },
        {
          to: "/docs/module-2/physics-simulation-gazebo",
          label: "Module 2",
          position: "left",
        },
        {
          to: "/docs/module-3/perception-localization",
          label: "Module 3",
          position: "left",
        },
        {
          href: "https://github.com/Najam-ul-islam/physical-ai-humanoid-robotics",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Book Modules",
          items: [
            {
              label: "Module 1: ROS2 & Robot Control",
              to: "/docs/module-1/ros2-fundamentals",
            },
            {
              label: "Module 2: Simulation & Perception",
              to: "/docs/module-2/physics-simulation-gazebo",
            },
            {
              label: "Module 3: Navigation & Planning",
              to: "/docs/module-3/perception-localization",
            },
          ],
        },
        {
          title: "Resources",
          items: [
            {
              label: "Research Papers",
              href: "/docs/module-4/cognitive-planning-llms",
            },
            {
              label: "Simulation Tools",
              href: "/docs/module-2/physics-simulation-gazebo",
            },
            {
              label: "Hardware Specifications",
              href: "/docs/module-1/urdf",
            },
          ],
        },
        {
          title: "Connect",
          items: [
            {
              label: "GitHub Repository",
              href: "https://github.com/Najam-ul-islam/physical-ai-humanoid-robotics",
            },
            {
              label: "Robotics Community",
              href: "https://forum.humanoid-robotics.org",
            },
            {
              label: "Academic Resources",
              href: "https://physical-ai.academia.edu",
            },
          ],
        },
      ],
      copyright: `Copyright © 2025 Physical AI & Humanoid Robotics Book. All rights reserved.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
