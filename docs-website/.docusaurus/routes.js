import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/BookProject/ur/',
    component: ComponentCreator('/BookProject/ur/', '9a6'),
    exact: true
  },
  {
    path: '/BookProject/ur/',
    component: ComponentCreator('/BookProject/ur/', '1bc'),
    routes: [
      {
        path: '/BookProject/ur/',
        component: ComponentCreator('/BookProject/ur/', '999'),
        routes: [
          {
            path: '/BookProject/ur/',
            component: ComponentCreator('/BookProject/ur/', '250'),
            routes: [
              {
                path: '/BookProject/ur/capstone/intro',
                component: ComponentCreator('/BookProject/ur/capstone/intro', '607'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/BookProject/ur/digital-twin/intro',
                component: ComponentCreator('/BookProject/ur/digital-twin/intro', '3bd'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/BookProject/ur/nervous-system/intro',
                component: ComponentCreator('/BookProject/ur/nervous-system/intro', '1a2'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/BookProject/ur/robot-brain/intro',
                component: ComponentCreator('/BookProject/ur/robot-brain/intro', '219'),
                exact: true,
                sidebar: "mainSidebar"
              },
              {
                path: '/BookProject/ur/the-mind/intro',
                component: ComponentCreator('/BookProject/ur/the-mind/intro', '182'),
                exact: true,
                sidebar: "mainSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
