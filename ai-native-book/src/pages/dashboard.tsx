import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useHistory } from '@docusaurus/router';
import styles from './Dashboard.module.css';

function DashboardPage() {
  const history = useHistory ? useHistory() : null;
  const navigate = (path: string) => {
    if (history) {
      history.push(path);
    } else {
      window.location.href = path;
    }
  };
  const [user, setUser] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState('');

  useEffect(() => {
    const fetchUser = async () => {
      try {
        const token = localStorage.getItem('authToken');
        if (!token) {
          navigate('/login');
          return;
        }

        const backendUrl = 'http://localhost:5000';
        const response = await fetch(`${backendUrl}/api/users/profile`, {
          method: 'GET',
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
        });

        if (!response.ok) {
          throw new Error('Failed to fetch user profile');
        }

        const data = await response.json();
        setUser(data.user);
      } catch (err: any) {
        setError(err.message || 'Failed to load user data');
        navigate('/login');
      } finally {
        setLoading(false);
      }
    };

    fetchUser();
  }, []);

  const handleLogout = async () => {
    try {
      const backendUrl = 'http://localhost:5000';
      await fetch(`${backendUrl}/api/auth/logout`, { method: 'POST' });
      localStorage.removeItem('authToken');
      navigate('/');
    } catch (err) {
      console.error('Logout error:', err);
    }
  };

  if (loading) {
    return (
      <Layout title="Dashboard" description="Your personalized dashboard">
        <main className={styles.dashboardSection}>
          <div className="container">
            <p>Loading...</p>
          </div>
        </main>
      </Layout>
    );
  }

  if (error) {
    return (
      <Layout title="Dashboard" description="Your personalized dashboard">
        <main className={styles.dashboardSection}>
          <div className="container">
            <p className={styles.error}>{error}</p>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title="Dashboard" description="Your personalized dashboard">
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container">
          <h1 className="hero__title">Welcome, {user?.name || 'User'}!</h1>
          <p className="hero__subtitle">Your personalized dashboard</p>
        </div>
      </header>
      <main className={styles.dashboardSection}>
        <div className={clsx('container', styles.dashboardContainer)}>
          <div className={styles.profileCard}>
            <h2>Your Profile</h2>
            <div className={styles.profileInfo}>
              {user?.image && (
                <img
                  src={user.image}
                  alt={user.name}
                  className={styles.profileImage}
                />
              )}
              <div className={styles.userDetails}>
                <p>
                  <strong>Name:</strong> {user?.name}
                </p>
                <p>
                  <strong>Email:</strong> {user?.email}
                </p>
                <p>
                  <strong>Provider:</strong> {user?.provider}
                </p>
                <p>
                  <strong>Joined:</strong>{' '}
                  {user?.createdAt
                    ? new Date(user.createdAt).toLocaleDateString()
                    : 'N/A'}
                </p>
              </div>
            </div>

            <div className={styles.actions}>
              <Link to="/profile" className="button button--primary">
                Edit Profile
              </Link>
              <button
                className="button button--danger"
                onClick={handleLogout}
              >
                Logout
              </button>
            </div>
          </div>

          <div className={styles.contentCard}>
            <h2>Your Content</h2>
            <p>Access all your saved content and settings here.</p>
            <Link to="/" className="button button--secondary">
              Back to Home
            </Link>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default DashboardPage;
