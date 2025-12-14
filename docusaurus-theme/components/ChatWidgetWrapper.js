import React from 'react';
import DocusaurusChat from './DocusaurusChat';

// Simple wrapper component for the chat widget that can be used in Docusaurus
const ChatWidgetWrapper = () => {
  return (
    <div className="chat-widget-container">
      <DocusaurusChat />
    </div>
  );
};

export default ChatWidgetWrapper;