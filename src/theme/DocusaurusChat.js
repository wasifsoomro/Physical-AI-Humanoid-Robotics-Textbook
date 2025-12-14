// JavaScript wrapper for the Docusaurus Chat functionality
import React, { useState, useEffect, useRef } from 'react';
import { v4 as uuidv4 } from 'uuid';

// Since we can't directly import the TypeScript React component in Docusaurus,
// we'll dynamically load the built chat widget or create a React component
// that mimics the functionality of the ChatWidget.tsx

const DocusaurusChat = () => {
  // State for the chat functionality
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false); // For collapsible chat
  const [isDarkMode, setIsDarkMode] = useState(false); // For dark mode detection
  const [sessionId, setSessionId] = useState(null);
  const messagesEndRef = useRef(null);
  const chatContainerRef = useRef(null);

  // Initialize session ID on client side only
  useEffect(() => {
    // Only run on client side
    if (typeof window !== 'undefined') {
      let id = localStorage.getItem('chat-session-id');
      if (!id) {
        id = uuidv4();
        localStorage.setItem('chat-session-id', id);
      }
      setSessionId(id);
    }
  }, []);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages, isLoading]);

  // Detect and monitor dark mode
  useEffect(() => {
    const updateDarkMode = () => {
      const isDark = document.documentElement.getAttribute('data-theme') === 'dark' ||
        (!document.documentElement.getAttribute('data-theme') &&
         window.matchMedia('(prefers-color-scheme: dark)').matches);
      setIsDarkMode(isDark);
    };

    // Initial check
    updateDarkMode();

    // Listen for theme changes
    const observer = new MutationObserver(updateDarkMode);
    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['data-theme']
    });

    // Listen for system preference changes
    const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
    mediaQuery.addEventListener('change', updateDarkMode);

    return () => {
      observer.disconnect();
      mediaQuery.removeEventListener('change', updateDarkMode);
    };
  }, []);

  const scrollToBottom = () => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message to UI immediately
    const userMessage = {
      id: uuidv4(),
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request body for global mode only
      const requestBody = {
        session_id: sessionId,
        message: inputValue,
        mode: 'global', // Fixed to global mode only
      };

      // Send message to backend
      const response = await fetch('http://localhost:8002/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: uuidv4(),
          role: 'assistant',
          content: data.response,
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorData = await response.json();
        const errorMessage = {
          id: uuidv4(),
          role: 'assistant',
          content: `Error: ${errorData.detail || 'Failed to get response'}`,
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: uuidv4(),
        role: 'assistant',
        content: 'Error: Failed to connect to the chat service',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Toggle chat window open/close
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Define theme colors based on dark mode
  const themeColors = isDarkMode
    ? {
        primary: '#25c2a0',
        background: '#1a1a1a',
        surface: '#2d2d2d',
        text: '#ffffff',
        textSecondary: '#b0b0b0',
        border: '#444',
        userMessageBg: '#3a3a3a',
        assistantMessageBg: '#2d2d2d',
        inputBg: '#3a3a3a',
        inputBorder: '#555',
        placeholder: '#888'
      }
    : {
        primary: '#25c2a0',
        background: '#ffffff',
        surface: '#fafafa',
        text: '#000000',
        textSecondary: '#666666',
        border: '#ccc',
        userMessageBg: '#e3f2fd',
        assistantMessageBg: '#ffffff',
        inputBg: '#ffffff',
        inputBorder: '#ccc',
        placeholder: '#999999'
      };

  // Don't render anything until session ID is available (client-side only)
  if (!sessionId) {
    return null;
  }

  // If chat is closed, show only the logo/button
  if (!isOpen) {
    return (
      <div
        onClick={toggleChat}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: themeColors.primary,
          color: 'white',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          cursor: 'pointer',
          boxShadow: '0 4px 12px rgba(0, 0, 0, 0.2)',
          zIndex: 1000,
          fontSize: '24px',
          fontWeight: 'bold',
          border: 'none'
        }}
        aria-label="Open chat"
        role="button"
      >
        🤖
      </div>
    );
  }

  // If chat is open, show the full chat interface
  return (
    <div style={{
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      width: '400px',
      height: '500px',
      border: `1px solid ${themeColors.border}`,
      borderRadius: '12px',
      overflow: 'hidden',
      backgroundColor: themeColors.background,
      display: 'flex',
      flexDirection: 'column',
      boxShadow: '0 8px 24px rgba(0, 0, 0, 0.2)',
      fontFamily: 'system-ui, -apple-system, sans-serif',
      zIndex: 1000
    }}>
      {/* Header with close button */}
      <div style={{
        backgroundColor: themeColors.primary,
        color: 'white',
        padding: '12px 16px',
        fontWeight: 'bold',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center'
      }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
          <span style={{ fontSize: '18px' }}>🤖</span>
          <span>Humanoid Robotics Assistant</span>
        </div>
        <div style={{ display: 'flex', gap: '8px' }}>
          <button
            onClick={toggleChat}
            style={{
              background: 'rgba(255,255,255,0.2)',
              color: 'white',
              border: '1px solid rgba(255,255,255,0.3)',
              borderRadius: '4px',
              padding: '2px 8px',
              fontSize: '12px',
              cursor: 'pointer'
            }}
          >
            ✕
          </button>
        </div>
      </div>

      {/* Messages container with auto-scroll */}
      <div
        ref={chatContainerRef}
        style={{
          flex: 1,
          padding: '12px',
          overflowY: 'auto',
          display: 'flex',
          flexDirection: 'column',
          gap: '8px',
          backgroundColor: themeColors.surface
        }}
      >
        {messages.length === 0 ? (
          <div style={{
            textAlign: 'center',
            color: themeColors.textSecondary,
            fontStyle: 'italic',
            padding: '20px 0',
            flex: 1,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center'
          }}>
            <div>
              <div style={{ fontSize: '24px', marginBottom: '10px', color: themeColors.primary }}>🤖</div>
              <div style={{ color: themeColors.text }}>Ask me anything about the Humanoid Robotics Book!</div>
              <div style={{ fontSize: '12px', marginTop: '8px', color: themeColors.textSecondary }}>
                Global context mode enabled
              </div>
            </div>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              style={{
                alignSelf: message.role === 'user' ? 'flex-end' : 'flex-start',
                backgroundColor: message.role === 'user' ? themeColors.userMessageBg : themeColors.assistantMessageBg,
                padding: '10px 14px',
                borderRadius: message.role === 'user' ? '12px 12px 0 12px' : '12px 12px 12px 0',
                maxWidth: '85%',
                wordWrap: 'break-word',
                boxShadow: '0 1px 3px rgba(0,0,0,0.1)',
                color: themeColors.text
              }}
            >
              <div style={{ fontSize: '14px', lineHeight: '1.4' }}>
                {message.content}
              </div>
              <div style={{
                fontSize: '10px',
                color: themeColors.textSecondary,
                textAlign: 'right',
                marginTop: '4px'
              }}>
                {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div
            style={{
              alignSelf: 'flex-start',
              backgroundColor: themeColors.assistantMessageBg,
              padding: '10px 14px',
              borderRadius: '12px 12px 12px 0',
              maxWidth: '85%',
              boxShadow: '0 1px 3px rgba(0,0,0,0.1)',
              color: themeColors.text
            }}
          >
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
              <div>Thinking</div>
              <div style={{ display: 'flex', gap: '2px' }}>
                <span>.</span>
                <span>.</span>
                <span>.</span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input form */}
      <form onSubmit={handleSubmit} style={{
        padding: '12px',
        borderTop: `1px solid ${themeColors.border}`,
        backgroundColor: themeColors.background,
        display: 'flex',
        alignItems: 'center'
      }}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about the book..."
          style={{
            flex: 1,
            padding: '12px 15px',
            border: `1px solid ${themeColors.inputBorder}`,
            borderRadius: '24px',
            fontSize: '14px',
            marginRight: '10px',
            outline: 'none',
            backgroundColor: themeColors.inputBg,
            color: themeColors.text
          }}
          disabled={isLoading}
          autoFocus
        />
        <button
          type="submit"
          style={{
            padding: '12px 20px',
            backgroundColor: themeColors.primary,
            color: 'white',
            border: 'none',
            borderRadius: '24px',
            cursor: isLoading ? 'not-allowed' : 'pointer',
            fontSize: '14px',
            fontWeight: '600',
            minWidth: '60px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center'
          }}
          disabled={isLoading || !inputValue.trim()}
        >
          {isLoading ? '...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default DocusaurusChat;