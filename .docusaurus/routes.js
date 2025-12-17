import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '6eb'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '58f'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '0d7'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'aa8'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '1cb'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '6b3'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '45a'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '503'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'f87'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '2d2'),
            routes: [
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', 'aed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/chapter1',
                component: ComponentCreator('/docs/module1/chapter1', '899'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/chapter2',
                component: ComponentCreator('/docs/module1/chapter2', '01c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/chapter3',
                component: ComponentCreator('/docs/module1/chapter3', '1b6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/mini-project',
                component: ComponentCreator('/docs/module1/mini-project', '55b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/practical-demos',
                component: ComponentCreator('/docs/module1/practical-demos', 'b1f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/quiz',
                component: ComponentCreator('/docs/module1/quiz', 'd19'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/summary',
                component: ComponentCreator('/docs/module1/summary', 'b64'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter1',
                component: ComponentCreator('/docs/module2/chapter1', 'e7f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter2',
                component: ComponentCreator('/docs/module2/chapter2', 'ba8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter3',
                component: ComponentCreator('/docs/module2/chapter3', 'b82'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/mini-project',
                component: ComponentCreator('/docs/module2/mini-project', 'c34'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/practical-demos',
                component: ComponentCreator('/docs/module2/practical-demos', 'ec1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/quiz',
                component: ComponentCreator('/docs/module2/quiz', '1ee'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/summary',
                component: ComponentCreator('/docs/module2/summary', '869'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/chapter1',
                component: ComponentCreator('/docs/module3/chapter1', 'ce2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/chapter2',
                component: ComponentCreator('/docs/module3/chapter2', 'db1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/chapter3',
                component: ComponentCreator('/docs/module3/chapter3', '491'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/chapter4',
                component: ComponentCreator('/docs/module3/chapter4', 'f23'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/mini-project',
                component: ComponentCreator('/docs/module3/mini-project', '5d9'),
                exact: true
              },
              {
                path: '/docs/module3/practical-demos',
                component: ComponentCreator('/docs/module3/practical-demos', 'a60'),
                exact: true
              },
              {
                path: '/docs/module3/quiz',
                component: ComponentCreator('/docs/module3/quiz', 'ed5'),
                exact: true
              },
              {
                path: '/docs/module3/summary',
                component: ComponentCreator('/docs/module3/summary', '9f4'),
                exact: true
              },
              {
                path: '/docs/module4/chapter1',
                component: ComponentCreator('/docs/module4/chapter1', '645'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter2',
                component: ComponentCreator('/docs/module4/chapter2', '68e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter3',
                component: ComponentCreator('/docs/module4/chapter3', '6d1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter4',
                component: ComponentCreator('/docs/module4/chapter4', '104'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/mini-project',
                component: ComponentCreator('/docs/module4/mini-project', '5d6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/practical-demos',
                component: ComponentCreator('/docs/module4/practical-demos', '7b4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/quiz',
                component: ComponentCreator('/docs/module4/quiz', '563'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/summary',
                component: ComponentCreator('/docs/module4/summary', '354'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/rag-chatbot/',
                component: ComponentCreator('/docs/rag-chatbot/', '7b7'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '36e'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
