import React, { useState } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useHistory } from '@docusaurus/router';
import styles from './Login.module.css';

function LoginPage() {
  const history = useHistory ? useHistory() : null;
  const navigate = (path) => {
    if (history) {
      history.push(path);
    } else {
      window.location.href = path;
    }
  };
  
  const [formData, setFormData] = useState({
    email: '',
    password: '',
  });
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const backendUrl = 'http://localhost:5000';
      const response = await fetch(`${backendUrl}/api/auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Login failed');
      }

      // Store token and user data in localStorage for client-side use
      if (data.token) {
        localStorage.setItem('authToken', data.token);
        localStorage.setItem('user', JSON.stringify(data.user));
        
        // Dispatch auth change event to update navbar
        window.dispatchEvent(new Event('authChange'));
      }

      // Redirect to dashboard
      navigate('/dashboard');
    } catch (err: any) {
      setError(err.message || 'An error occurred during login');
      console.error('Login error:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleGoogleSignIn = async () => {
    try {
      // Redirect to NextAuth Google sign-in
      window.location.href = '/api/auth/signin/google';
    } catch (err) {
      console.error('Google sign-in error:', err);
      setError('Google sign-in failed');
    }
  };

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
              {error && <div className={styles.errorMessage}>{error}</div>}
              <form onSubmit={handleSubmit}>
                <div className={styles.formGroup}>
                  <label htmlFor="email">Email</label>
                  <input
                    type="email"
                    id="email"
                    name="email"
                    placeholder="Enter your email"
                    value={formData.email}
                    onChange={handleChange}
                    required
                  />
                </div>
                <div className={styles.formGroup}>
                  <label htmlFor="password">Password</label>
                  <input
                    type="password"
                    id="password"
                    name="password"
                    placeholder="Enter your password"
                    value={formData.password}
                    onChange={handleChange}
                    required
                  />
                </div>
                <button
                  type="submit"
                  className="button button--primary button--block"
                  disabled={loading}
                >
                  {loading ? 'Signing In...' : 'Sign In'}
                </button>
              </form>

              <div className={styles.divider}>OR</div>

              <button
                type="button"
                className={clsx('button button--outline button--block', styles.googleButton)}
                onClick={handleGoogleSignIn}
              >
                🔐 Sign In with Google
              </button>

              <p className={styles.signupLink}>Don't have an account? <Link to="/signup">Sign Up</Link></p>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}

export default LoginPage;
