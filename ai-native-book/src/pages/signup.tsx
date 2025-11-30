import React from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import Link from '@docusaurus/Link'; // Import Link for internal navigation
import styles from './Signup.module.css'; // Use a dedicated CSS module for Signup, reusing common styles

function SignupPage() {
  return (
    <Layout
      title="Sign Up"
      description="Create a new account"
    >
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container">
          <h1 className="hero__title">Sign Up</h1>
          <p className="hero__subtitle">Join us and access exclusive content</p>
        </div>
      </header>
      <main>
        <section className={styles.signupSection}>
          <div className={clsx('container', styles.signupContainer)}>
            <div className={styles.signupForm}>
              <h2>Create Your Account</h2>
              <form>
                <div className={styles.formGroup}>
                  <label htmlFor="fullName">Full Name</label>
                  <input type="text" id="fullName" name="fullName" placeholder="Enter your full name" required />
                </div>
                <div className={styles.formGroup}>
                  <label htmlFor="email">Email</label>
                  <input type="email" id="email" name="email" placeholder="Enter your email" required />
                </div>
                <div className={styles.formGroup}>
                  <label htmlFor="password">Password</label>
                  <input type="password" id="password" name="password" placeholder="Create a password" required />
                </div>
                <div className={styles.formGroup}>
                  <label htmlFor="confirmPassword">Confirm Password</label>
                  <input type="password" id="confirmPassword" name="confirmPassword" placeholder="Confirm your password" required />
                </div>
                <button type="submit" className="button button--primary button--block">Create Account</button>
              </form>
              <p className={styles.signinLink}>Already have an account? <Link to="/login">Sign In</Link></p>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}

export default SignupPage;
