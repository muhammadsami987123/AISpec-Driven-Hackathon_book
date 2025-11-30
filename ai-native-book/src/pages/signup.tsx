import React, { useState } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useHistory } from '@docusaurus/router';
import styles from './Signup.module.css';

function SignupPage() {
  const history = useHistory ? useHistory() : null;
  const navigate = (path) => {
    if (history) {
      history.push(path);
    } else {
      window.location.href = path;
    }
  };
  
  const [formData, setFormData] = useState({
    fullName: '',
    email: '',
    password: '',
    confirmPassword: '',
  });
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState(false);
  const [successMessage, setSuccessMessage] = useState('');

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
    // Clear error for this field when user starts typing
    if (errors[name]) {
      setErrors((prev) => ({
        ...prev,
        [name]: '',
      }));
    }
  };

  const validateForm = (): boolean => {
    const newErrors: Record<string, string> = {};

    if (!formData.fullName.trim()) {
      newErrors.fullName = 'Full name is required';
    } else if (formData.fullName.trim().length < 2) {
      newErrors.fullName = 'Full name must be at least 2 characters';
    }

    if (!formData.email.trim()) {
      newErrors.email = 'Email is required';
    } else if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(formData.email)) {
      newErrors.email = 'Invalid email address';
    }

    if (!formData.password) {
      newErrors.password = 'Password is required';
    } else if (formData.password.length < 8) {
      newErrors.password = 'Password must be at least 8 characters';
    } else if (!/[A-Z]/.test(formData.password)) {
      newErrors.password = 'Password must contain at least one uppercase letter';
    } else if (!/[0-9]/.test(formData.password)) {
      newErrors.password = 'Password must contain at least one number';
    }

    if (formData.password !== formData.confirmPassword) {
      newErrors.confirmPassword = 'Passwords do not match';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setSuccessMessage('');
    setErrors({});

    if (!validateForm()) {
      return;
    }

    setLoading(true);

    try {
      const backendUrl = 'http://localhost:5000';
      const response = await fetch(`${backendUrl}/api/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          fullName: formData.fullName,
          email: formData.email,
          password: formData.password,
          confirmPassword: formData.confirmPassword,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        if (data.error === 'Email already registered') {
          setErrors({ email: 'This email is already registered' });
        } else if (data.details) {
          const fieldErrors: Record<string, string> = {};
          data.details.forEach((detail: any) => {
            fieldErrors[detail.path[0]] = detail.message;
          });
          setErrors(fieldErrors);
        } else {
          setErrors({ submit: data.error || 'Signup failed' });
        }
        return;
      }

      // Store token and user data
      localStorage.setItem('authToken', data.token);
      localStorage.setItem('user', JSON.stringify(data.user));
      
      // Dispatch auth change event to update navbar
      window.dispatchEvent(new Event('authChange'));
      
      setSuccessMessage('Account created successfully! Redirecting to dashboard...');
      // Redirect to dashboard after 2 seconds
      setTimeout(() => {
        navigate('/dashboard');
      }, 2000);
    } catch (err: any) {
      setErrors({ submit: err.message || 'An error occurred during signup' });
      console.error('Signup error:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleGoogleSignUp = async () => {
    try {
      // Redirect to NextAuth Google sign-in
      window.location.href = '/api/auth/signin/google';
    } catch (err) {
      console.error('Google sign-up error:', err);
      setErrors({ submit: 'Google sign-up failed' });
    }
  };

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
              {errors.submit && <div className={styles.errorMessage}>{errors.submit}</div>}
              {successMessage && <div className={styles.successMessage}>{successMessage}</div>}
              <form onSubmit={handleSubmit}>
                <div className={styles.formGroup}>
                  <label htmlFor="fullName">Full Name</label>
                  <input
                    type="text"
                    id="fullName"
                    name="fullName"
                    placeholder="Enter your full name"
                    value={formData.fullName}
                    onChange={handleChange}
                    required
                  />
                  {errors.fullName && <span className={styles.fieldError}>{errors.fullName}</span>}
                </div>
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
                  {errors.email && <span className={styles.fieldError}>{errors.email}</span>}
                </div>
                <div className={styles.formGroup}>
                  <label htmlFor="password">Password</label>
                  <input
                    type="password"
                    id="password"
                    name="password"
                    placeholder="Create a password (min 8 chars, 1 uppercase, 1 number)"
                    value={formData.password}
                    onChange={handleChange}
                    required
                  />
                  {errors.password && <span className={styles.fieldError}>{errors.password}</span>}
                </div>
                <div className={styles.formGroup}>
                  <label htmlFor="confirmPassword">Confirm Password</label>
                  <input
                    type="password"
                    id="confirmPassword"
                    name="confirmPassword"
                    placeholder="Confirm your password"
                    value={formData.confirmPassword}
                    onChange={handleChange}
                    required
                  />
                  {errors.confirmPassword && <span className={styles.fieldError}>{errors.confirmPassword}</span>}
                </div>
                <button
                  type="submit"
                  className="button button--primary button--block"
                  disabled={loading}
                >
                  {loading ? 'Creating Account...' : 'Create Account'}
                </button>
              </form>

              <div className={styles.divider}>OR</div>

              <button
                type="button"
                className={clsx('button button--outline button--block', styles.googleButton)}
                onClick={handleGoogleSignUp}
              >
                🔐 Sign Up with Google
              </button>

              <p className={styles.signinLink}>Already have an account? <Link to="/login">Sign In</Link></p>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}

export default SignupPage;
