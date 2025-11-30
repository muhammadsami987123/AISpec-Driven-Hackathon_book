import React from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import Link from '@docusaurus/Link'; // Import Link for internal navigation
import styles from './Login.module.css';

function LoginPage() {
  return (
    <Layout
      title="Sign In"
      description="Login to your account"
    >
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container">
          <h1 className="hero__title">Sign In</h1>
          <p className="hero__subtitle">Access your personalized content</p>
        </div>
      </header>
      <main>
        <section className={styles.loginSection}>
          <div className={clsx('container', styles.loginContainer)}>
            <div className={styles.loginForm}>
              <h2>Welcome Back!</h2>
              <form>
                <div className={styles.formGroup}>
                  <label htmlFor="username">Username</label>
                  <input type="text" id="username" name="username" placeholder="Enter your username" required />
                </div>
                <div className={styles.formGroup}>
                  <label htmlFor="password">Password</label>
                  <input type="password" id="password" name="password" placeholder="Enter your password" required />
                </div>
                <button type="submit" className="button button--primary button--block">Sign In</button>
              </form>
              <p className={styles.signupLink}>Don't have an account? <Link to="/signup">Sign Up</Link></p>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}

export default LoginPage;
