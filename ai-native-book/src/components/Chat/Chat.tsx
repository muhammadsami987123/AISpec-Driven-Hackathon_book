import React, { useState, useRef, useEffect } from "react";
import styles from "./Chat.module.css";

interface Message {
  id: string;
  type: "user" | "assistant";
  content: string;
  sources?: Array<{
    title: string;
    content: string;
    score: number;
  }>;
  timestamp: Date;
}

interface ChatProps {
  onSendMessage?: (message: string) => void;
}

const Chat: React.FC<ChatProps> = () => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);
  const [showSources, setShowSources] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const getAuthToken = () => {
    return typeof window !== "undefined" ? localStorage.getItem("authToken") : null;
  };

  const sendMessage = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!input.trim()) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      type: "user",
      content: input,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInput("");
    setLoading(true);
    setError(null);

    try {
      const token = getAuthToken();
      if (!token) {
        throw new Error("Not authenticated. Please log in.");
      }

      const response = await fetch("http://localhost:5000/api/rag/chat", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Authorization: `Bearer ${token}`,
        },
        body: JSON.stringify({
          query: input,
          include_sources: true,
          search_limit: 5,
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.statusText}`);
      }

      const data = await response.json();

      // Add assistant message
      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        type: "assistant",
        content: data.response,
        sources: data.sources || [],
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : "Failed to send message";
      setError(errorMessage);

      // Add error message to chat
      const errorChatMessage: Message = {
        id: (Date.now() + 1).toString(),
        type: "assistant",
        content: `Sorry, an error occurred: ${errorMessage}`,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorChatMessage]);
    } finally {
      setLoading(false);
    }
  };

  const formatTime = (date: Date) => {
    return date.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
  };

  return (
    <div className={styles.chatContainer}>
      {/* Header */}
      <div className={styles.header}>
        <h2>AI Assistant</h2>
        <p>Ask questions about the AI-Native Book</p>
      </div>

      {/* Messages */}
      <div className={styles.messagesWrapper}>
        {messages.length === 0 ? (
          <div className={styles.emptyState}>
            <div className={styles.emptyIcon}>💬</div>
            <h3>Start a Conversation</h3>
            <p>Ask me anything about the AI-Native Book and I'll search through the content to provide you with relevant answers.</p>
          </div>
        ) : (
          <div className={styles.messages}>
            {messages.map((message) => (
              <div key={message.id} className={`${styles.message} ${styles[message.type]}`}>
                <div className={styles.messageContent}>
                  <div className={styles.bubble}>
                    <p>{message.content}</p>
                  </div>

                  {/* Sources */}
                  {message.sources && message.sources.length > 0 && (
                    <div className={styles.sourcesToggle}>
                      <button
                        onClick={() => setShowSources(!showSources)}
                        className={styles.sourcesBtn}
                      >
                        📚 {message.sources.length} source{message.sources.length !== 1 ? "s" : ""}
                      </button>

                      {showSources && (
                        <div className={styles.sources}>
                          {message.sources.map((source, idx) => (
                            <div key={idx} className={styles.source}>
                              <div className={styles.sourceTitle}>
                                {source.title}
                                <span className={styles.score}>
                                  {(source.score * 100).toFixed(0)}%
                                </span>
                              </div>
                              <p className={styles.sourceContent}>{source.content}...</p>
                            </div>
                          ))}
                        </div>
                      )}
                    </div>
                  )}

                  <span className={styles.time}>{formatTime(message.timestamp)}</span>
                </div>
              </div>
            ))}

            {loading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.messageContent}>
                  <div className={styles.bubble}>
                    <div className={styles.typingIndicator}>
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>
        )}
      </div>

      {/* Error Alert */}
      {error && (
        <div className={styles.errorAlert}>
          <span>⚠️ {error}</span>
          <button onClick={() => setError(null)}>×</button>
        </div>
      )}

      {/* Input Form */}
      <form onSubmit={sendMessage} className={styles.inputForm}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder="Ask a question about the book..."
          disabled={loading}
          className={styles.input}
        />
        <button type="submit" disabled={loading || !input.trim()} className={styles.sendBtn}>
          {loading ? "Sending..." : "Send"}
        </button>
      </form>
    </div>
  );
};

export default Chat;
