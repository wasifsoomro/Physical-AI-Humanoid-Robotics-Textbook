import React from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme-original/Layout';
import DocusaurusChat from '../components/DocusaurusChat';

export default function LayoutWrapper(props) {
  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();

  return (
    <>
      <Layout {...props} />
      <div style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        zIndex: 10000, // Increased z-index to ensure it's on top
        width: '400px',
        height: '500px',
      }}>
        <DocusaurusChat />
      </div>
    </>
  );
}