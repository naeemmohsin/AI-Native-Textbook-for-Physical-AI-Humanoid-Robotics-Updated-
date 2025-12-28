/**
 * ChatWidget - Floating chatbot for the ROS 2 Robotics book.
 *
 * Allows users to ask questions and receive AI-generated answers
 * grounded in book content with source citations.
 */

import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

// =============================================================================
// Types
// =============================================================================

interface Citation {
  title: string;
  url: string;
}

interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
}

interface QueryResponse {
  response: string;
  citations: Citation[];
  session_id: string;
}

// =============================================================================
// Constants
// =============================================================================

const API_URL = 'http://localhost:8000';

// =============================================================================
// Component
// =============================================================================

export default function ChatWidget(): React.ReactElement {
  // T012: Chat state management
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // T021: Session ID from localStorage
  const [sessionId, setSessionId] = useState<string>(() => {
    if (typeof window !== 'undefined') {
      const stored = localStorage.getItem('chat_session_id');
      if (stored) return stored;
      const newId = `session_${Math.random().toString(36).substring(2, 10)}`;
      localStorage.setItem('chat_session_id', newId);
      return newId;
    }
    return '';
  });

  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  // T013: API call to POST /api/query
  const sendQuery = async (query: string, context?: string) => {
    setIsLoading(true);
    setError(null);

    // Add user message immediately
    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      role: 'user',
      content: query,
    };
    setMessages((prev) => [...prev, userMessage]);

    try {
      const response = await fetch(`${API_URL}/api/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          session_id: sessionId,
          context,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data: QueryResponse = await response.json();

      // Update session ID if returned
      if (data.session_id) {
        setSessionId(data.session_id);
        localStorage.setItem('chat_session_id', data.session_id);
      }

      // T014: Add assistant message with citations
      const assistantMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.response,
        citations: data.citations,
      };
      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to send message';
      setError(errorMessage);
    } finally {
      setIsLoading(false);
    }
  };

  // T016: Handle form submission with empty query validation
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    const trimmed = inputValue.trim();
    if (!trimmed) {
      return; // T016: Empty query validation
    }
    sendQuery(trimmed);
    setInputValue('');
  };

  // Toggle chat panel
  const toggleChat = () => {
    setIsOpen((prev) => !prev);
    setError(null);
  };

  return (
    <>
      {/* Floating chat button */}
      <button
        className={styles.chatButton}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3 className={styles.chatTitle}>Ask the Book</h3>
            <button className={styles.closeButton} onClick={toggleChat}>
              ✕
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.message + ' ' + styles.assistantMessage}>
                Hi! I can answer questions about the ROS 2 Robotics book.
                Try asking about ROS 2, Gazebo, URDF, or other topics!
              </div>
            )}

            {messages.map((msg) => (
              <div
                key={msg.id}
                className={`${styles.message} ${
                  msg.role === 'user' ? styles.userMessage : styles.assistantMessage
                }`}
              >
                {msg.content}

                {/* T014: Display citations */}
                {msg.citations && msg.citations.length > 0 && (
                  <div className={styles.citations}>
                    <strong>Sources:</strong>
                    {msg.citations.map((citation, idx) => (
                      <a
                        key={idx}
                        href={citation.url}
                        className={styles.citationLink}
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        📖 {citation.title}
                      </a>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {/* Loading indicator */}
            {isLoading && (
              <div className={styles.loading}>
                <div className={styles.loadingDots}>
                  <div className={styles.loadingDot}></div>
                  <div className={styles.loadingDot}></div>
                  <div className={styles.loadingDot}></div>
                </div>
                <span>Thinking...</span>
              </div>
            )}

            {/* Error message */}
            {error && (
              <div className={styles.error}>
                ⚠️ {error}
                <br />
                <small>Please check if the backend is running.</small>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input area */}
          <form className={styles.inputArea} onSubmit={handleSubmit}>
            <input
              type="text"
              className={styles.input}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question..."
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={isLoading || !inputValue.trim()}
            >
              Send
            </button>
          </form>
        </div>
      )}
    </>
  );
}
