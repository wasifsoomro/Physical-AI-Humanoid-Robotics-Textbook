import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/docs',
    component: ComponentCreator('/docs', '38d'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '2cc'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '8cb'),
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
