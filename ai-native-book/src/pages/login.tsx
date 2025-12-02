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
  const [fieldErrors, setFieldErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState(false);
  const [showPassword, setShowPassword] = useState(false);
  const [rememberMe, setRememberMe] = useState(true);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
    if (fieldErrors[name]) {
      setFieldErrors((prev) => ({ ...prev, [name]: '' }));
    }
  };

  const validateForm = () => {
    const errs: Record<string, string> = {};
    if (!formData.email.trim()) {
      errs.email = 'Email is required';
    } else if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(formData.email)) {
      errs.email = 'Enter a valid email address';
    }
    if (!formData.password) {
      errs.password = 'Password is required';
    } else if (formData.password.length < 6) {
      errs.password = 'Password must be at least 6 characters';
    }
    setFieldErrors(errs);
    return Object.keys(errs).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setFieldErrors({});

    if (!validateForm()) {
      return;
    }
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
        const storage = rememberMe ? localStorage : sessionStorage;
        storage.setItem('authToken', data.token);
        storage.setItem('user', JSON.stringify(data.user));
        
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
                  {fieldErrors.email && (
                    <span className={styles.fieldError}>{fieldErrors.email}</span>
                  )}
                </div>
                <div className={clsx(styles.formGroup, styles.passwordField)}>
                  <label htmlFor="password">Password</label>
                  <div className={styles.passwordInputRow}>
                    <input
                      type={showPassword ? 'text' : 'password'}
                      id="password"
                      name="password"
                      placeholder="Enter your password"
                      value={formData.password}
                      onChange={handleChange}
                      required
                    />
                    <button
                      type="button"
                      className={styles.togglePassword}
                      onClick={() => setShowPassword((s) => !s)}
                      aria-label={showPassword ? 'Hide password' : 'Show password'}
                    >
                      {showPassword ? 'Hide' : 'Show'}
                    </button>
                  </div>
                  {fieldErrors.password && (
                    <span className={styles.fieldError}>{fieldErrors.password}</span>
                  )}
                </div>
                <div className={styles.helperRow}>
                  <label className={styles.rememberMe}>
                    <input
                      type="checkbox"
                      checked={rememberMe}
                      onChange={(e) => setRememberMe(e.target.checked)}
                    />
                    Remember me
                  </label>
                  <Link className={styles.forgotLink} to="/reset-password">Forgot password?</Link>
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
                disabled={loading}
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
