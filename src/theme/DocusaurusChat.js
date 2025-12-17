// JavaScript wrapper for the Docusaurus Chat functionality
import React, { useState, useEffect, useRef } from 'react';
import { v4 as uuidv4 } from 'uuid';
import { API_BASE_URL } from '@site/src/clientModules/config';

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
  const [selectedText, setSelectedText] = useState(''); // Currently selected text
  const [preservedSelectedText, setPreservedSelectedText] = useState(''); // Preserved selected text
  const [currentMode, setCurrentMode] = useState('global'); // 'global' or 'selected-text'
  const [isMobile, setIsMobile] = useState(false); // For mobile detection
  const currentModeRef = useRef(currentMode); // Ref to track current mode
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

  // Update ref when currentMode changes
  useEffect(() => {
    currentModeRef.current = currentMode;
  }, [currentMode]);

  // Detect mobile screen size
  useEffect(() => {
    const checkIsMobile = () => {
      if (typeof window !== 'undefined') {
        setIsMobile(window.innerWidth <= 768);
      }
    };

    // Initial check
    checkIsMobile();

    // Add resize listener
    window.addEventListener('resize', checkIsMobile);

    // Cleanup
    return () => {
      window.removeEventListener('resize', checkIsMobile);
    };
  }, []);

  // Handle text selection with preservation
  useEffect(() => {
    const handleSelection = () => {
      if (typeof window !== 'undefined' && typeof document !== 'undefined') {
        const currentSelection = window.getSelection().toString().trim();
        const hadPreviousSelection = preservedSelectedText && preservedSelectedText.trim() !== '';

        // Update the temporary selected text state
        setSelectedText(currentSelection);

        // Update preserved selected text based on current selection
        if (currentSelection) {
          // Always update preserved text when there's a non-empty selection
          // This ensures that when users select NEW text, it replaces the old preserved text
          setPreservedSelectedText(currentSelection);

          // Automatically switch to selected-text mode when text is selected
          if (currentModeRef.current !== 'selected-text') {
            setCurrentMode('selected-text');
          }
        }
        // Note: We don't clear preserved text on empty selection to allow for typing
        // The preserved text will be cleared when user explicitly switches to global mode
        // or presses Escape, or when they make a new selection
      }
    };

    const handleEscapeKey = (e) => {
      if (e.key === 'Escape') {
        // Clear selection when Escape is pressed
        window.getSelection().empty();
        setSelectedText('');
        setPreservedSelectedText(''); // Also clear preserved text
        setCurrentMode('global'); // Switch back to global mode
      }
    };

    // Add event listeners for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);
    document.addEventListener('keyup', handleEscapeKey);

    // Cleanup event listeners
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
      document.removeEventListener('keyup', handleEscapeKey);
    };
  }, []); // Remove currentMode from dependency to prevent re-adding listeners

  // Effect to provide visual feedback when in selected-text mode
  useEffect(() => {
    if (currentMode === 'selected-text' && preservedSelectedText) {
      // Optionally add other visual cues here if needed
    }
  }, [currentMode, preservedSelectedText]);

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
      // Prepare the request body with current mode and preserved selected text if applicable
      // Always send preservedSelectedText when in selected-text mode, even if current selection is empty
      const requestBody = {
        session_id: sessionId,
        message: inputValue,
        mode: currentMode,
        selected_text: currentMode === 'selected-text' ? preservedSelectedText : undefined,
      };

      // Send message to backend using the configured API base URL
      const response = await fetch(`${API_BASE_URL}/api/chat`, {
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

  // Switch between global and selected-text modes
  const switchMode = (mode) => {
    setCurrentMode(mode);
    // Clear preserved selected text when switching to global mode
    if (mode === 'global') {
      setPreservedSelectedText('');
    }
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
          bottom: isMobile ? '10px' : '20px',
          right: isMobile ? '10px' : '20px',
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
          border: 'none',
          // Ensure proper positioning on mobile
          maxWidth: '60px',
          maxHeight: '60px'
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
      bottom: isMobile ? '10px' : '20px',
      right: isMobile ? '10px' : '20px',
      width: isMobile ? 'calc(100% - 20px)' : '100%',
      maxWidth: isMobile ? '100%' : '400px',
      height: isMobile ? '60vh' : '70vh',
      minHeight: isMobile ? '300px' : '400px',
      maxHeight: isMobile ? 'none' : '500px',
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
      {/* Header with close button and mode selector */}
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
        <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
          {/* Mode selector */}
          <div style={{
            display: 'flex',
            gap: '2px',
            borderRadius: '12px',
            background: 'rgba(255,255,255,0.2)',
            padding: '2px'
          }}>
            <button
              onClick={() => switchMode('global')}
              style={{
                background: currentMode === 'global' ? 'white' : 'transparent',
                color: currentMode === 'global' ? themeColors.primary : 'white',
                border: 'none',
                borderRadius: '10px',
                padding: '4px 8px',
                fontSize: '12px',
                cursor: 'pointer',
                fontWeight: currentMode === 'global' ? 'bold' : 'normal'
              }}
            >
              Global
            </button>
            <button
              onClick={() => switchMode('selected-text')}
              style={{
                background: currentMode === 'selected-text' ? 'white' : 'transparent',
                color: currentMode === 'selected-text' ? themeColors.primary : 'white',
                border: 'none',
                borderRadius: '10px',
                padding: '4px 8px',
                fontSize: '12px',
                cursor: 'pointer',
                fontWeight: currentMode === 'selected-text' ? 'bold' : 'normal'
              }}
            >
              Selected Text
            </button>
          </div>
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
          backgroundColor: themeColors.surface,
          minHeight: '0' // Needed for flex child to have proper scrolling
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
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center'
          }}>
            <div>
              <div style={{ fontSize: '24px', marginBottom: '10px', color: themeColors.primary }}>🤖</div>
              <div style={{ color: themeColors.text }}>Ask me anything about the Humanoid Robotics Book!</div>
              <div style={{ fontSize: '12px', marginTop: '8px', color: themeColors.textSecondary }}>
                {currentMode === 'global'
                  ? 'Global context mode enabled'
                  : 'Selected text mode enabled'}
              </div>
              {currentMode === 'selected-text' && preservedSelectedText && (
                <div style={{
                  marginTop: '8px',
                  padding: '8px',
                  backgroundColor: themeColors.userMessageBg,
                  borderRadius: '8px',
                  fontSize: '12px',
                  color: themeColors.text,
                  maxWidth: '90%',
                  wordWrap: 'break-word',
                  maxHeight: '60px',
                  overflow: 'hidden',
                  textOverflow: 'ellipsis'
                }}>
                  Selected: "{preservedSelectedText.substring(0, 100)}{preservedSelectedText.length > 100 ? '...' : ''}"
                </div>
              )}
            </div>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              style={{
                alignSelf: message.role === 'user' ? 'flex-end' : 'flex-start',
                backgroundColor: message.role === 'user' ? themeColors.userMessageBg : themeColors.assistantMessageBg,
                padding: isMobile ? '8px 12px' : '10px 14px',
                borderRadius: message.role === 'user' ? '12px 12px 0 12px' : '12px 12px 12px 0',
                maxWidth: isMobile ? '80%' : '85%',
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
        alignItems: 'flex-end',
        flexDirection: 'column',
        gap: '8px'
      }}>
        <div style={{ display: 'flex', width: '100%', gap: '8px' }}>
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder={currentMode === 'selected-text' && preservedSelectedText
              ? "Ask about the selected text..."
              : "Ask a question about the book..."}
            style={{
              flex: 1,
              padding: isMobile ? '10px 12px' : '12px 15px',
              border: currentMode === 'selected-text' && preservedSelectedText
                ? `2px solid ${themeColors.primary}`  // Highlight when in selected-text mode
                : `1px solid ${themeColors.inputBorder}`,
              borderRadius: '24px',
              fontSize: isMobile ? '13px' : '14px',
              outline: 'none',
              backgroundColor: themeColors.inputBg,
              color: themeColors.text
            }}
            disabled={isLoading}
            autoFocus
          />
          {currentMode === 'selected-text' && preservedSelectedText && (
            <button
              type="button"
              onClick={async () => {
                // Auto-ask about the selected text with a general question
                const autoQuestion = "Can you explain this concept in more detail?";
                setInputValue(autoQuestion);

                // Prepare the request body with current mode and preserved selected text
                const requestBody = {
                  session_id: sessionId,
                  message: autoQuestion,
                  mode: currentMode,
                  selected_text: preservedSelectedText,
                };

                // Add user message to UI immediately
                const userMessage = {
                  id: uuidv4(),
                  role: 'user',
                  content: autoQuestion,
                  timestamp: new Date(),
                };

                setMessages(prev => [...prev, userMessage]);
                setIsLoading(true);

                try {
                  // Send message to backend using the configured API base URL
                  const response = await fetch(`${API_BASE_URL}/api/chat`, {
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
              }}
              style={{
                padding: isMobile ? '6px 10px' : '8px 12px',
                backgroundColor: themeColors.primary,
                color: 'white',
                border: 'none',
                borderRadius: '20px',
                cursor: 'pointer',
                fontSize: isMobile ? '11px' : '12px',
                fontWeight: 'bold',
                whiteSpace: 'nowrap'
              }}
              disabled={isLoading}
              title="Auto-explain selected text"
            >
              Auto
            </button>
          )}
        </div>
        <button
          type="submit"
          style={{
            padding: isMobile ? '10px 16px' : '12px 20px',
            backgroundColor: themeColors.primary,
            color: 'white',
            border: 'none',
            borderRadius: '24px',
            cursor: isLoading ? 'not-allowed' : 'pointer',
            fontSize: isMobile ? '13px' : '14px',
            fontWeight: '600',
            width: '100%',
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