import React, { useEffect, useState } from "react";
import Layout from "@theme/Layout";
import Chat from "../components/Chat/Chat";
import styles from "./chat.module.css";

export default function ChatPage() {
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Check if user is logged in
    const authToken = localStorage.getItem("authToken");
    if (authToken) {
      setIsLoggedIn(true);
    } else {
      setIsLoggedIn(false);
    }
    setLoading(false);
  }, []);

  if (loading) {
    return (
      <Layout title="AI Chat">
        <div className={styles.container}>
          <div className={styles.loading}>Loading...</div>
        </div>
      </Layout>
    );
  }

  if (!isLoggedIn) {
    return (
      <Layout title="AI Chat">
        <div className={styles.container}>
          <div className={styles.notLoggedIn}>
            <div className={styles.icon}>🔒</div>
            <h2>Sign In Required</h2>
            <p>Please sign in to use the AI Assistant and chat with your book content.</p>
            <a href="/login" className={styles.loginBtn}>
              Sign In
            </a>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="AI Chat - AI-Native Book">
      <div className={styles.container}>
        <Chat />
      </div>
    </Layout>
  );
}
