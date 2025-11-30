import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useHistory } from '@docusaurus/router';
import styles from './Profile.module.css';

interface User {
  id: string;
  fullName: string;
  email: string;
  image?: string;
  createdAt?: string;
}

function ProfilePage() {
  const history = useHistory ? useHistory() : null;
  const navigate = (path: string) => {
    if (history) {
      history.push(path);
    } else {
      window.location.href = path;
    }
  };

  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState('');
  const [editMode, setEditMode] = useState(false);
  const [formData, setFormData] = useState({
    fullName: '',
    email: '',
  });

  useEffect(() => {
    const checkAuth = async () => {
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
          throw new Error('Failed to fetch profile');
        }

        const data = await response.json();
        setUser(data.user);
        setFormData({
          fullName: data.user.fullName,
          email: data.user.email,
        });
      } catch (err: any) {
        setError(err.message || 'Failed to load profile');
        navigate('/login');
      } finally {
        setLoading(false);
      }
    };

    checkAuth();
  }, []);

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
  };

  const handleSave = async () => {
    try {
      const token = localStorage.getItem('authToken');
      const backendUrl = 'http://localhost:5000';
      const response = await fetch(`${backendUrl}/api/users/profile`, {
        method: 'PUT',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
      });

      if (!response.ok) {
        throw new Error('Failed to update profile');
      }

      const data = await response.json();
      setUser(data.user);
      localStorage.setItem('user', JSON.stringify(data.user));
      setEditMode(false);
      setError('');
    } catch (err: any) {
      setError(err.message || 'Failed to update profile');
    }
  };

  if (loading) {
    return (
      <Layout title="Profile" description="Your profile">
        <main className={styles.profileSection}>
          <div className="container">
            <p>Loading...</p>
          </div>
        </main>
      </Layout>
    );
  }

  if (error) {
    return (
      <Layout title="Profile" description="Your profile">
        <main className={styles.profileSection}>
          <div className="container">
            <p className={styles.error}>{error}</p>
          </div>
        </main>
      </Layout>
    );
  }

  if (!user) {
    return (
      <Layout title="Profile" description="Your profile">
        <main className={styles.profileSection}>
          <div className="container">
            <p>No user data found</p>
          </div>
        </main>
      </Layout>
    );
  }

  const initials = user.fullName
    .split(' ')
    .map((n) => n[0])
    .join('')
    .toUpperCase()
    .slice(0, 2);

  return (
    <Layout title="My Profile" description="Manage your profile">
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container">
          <h1 className="hero__title">My Profile</h1>
          <p className="hero__subtitle">View and manage your account information</p>
        </div>
      </header>
      <main className={styles.profileSection}>
        <div className={clsx('container', styles.profileContainer)}>
          <div className={styles.profileCard}>
            <div className={styles.profileHeader}>
              {user.image ? (
                <img src={user.image} alt={user.fullName} className={styles.profileImage} />
              ) : (
                <div className={styles.avatarLarge}>{initials}</div>
              )}
              <div className={styles.profileBasic}>
                <h2>{user.fullName}</h2>
                <p className={styles.email}>{user.email}</p>
                {user.createdAt && (
                  <p className={styles.joinDate}>
                    Joined {new Date(user.createdAt).toLocaleDateString()}
                  </p>
                )}
              </div>
            </div>

            {error && <div className={styles.error}>{error}</div>}

            <div className={styles.profileDetails}>
              <div className={styles.formGroup}>
                <label htmlFor="fullName">Full Name</label>
                {editMode ? (
                  <input
                    type="text"
                    id="fullName"
                    name="fullName"
                    value={formData.fullName}
                    onChange={handleInputChange}
                  />
                ) : (
                  <p className={styles.staticField}>{user.fullName}</p>
                )}
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="email">Email Address</label>
                {editMode ? (
                  <input
                    type="email"
                    id="email"
                    name="email"
                    value={formData.email}
                    onChange={handleInputChange}
                    disabled
                  />
                ) : (
                  <p className={styles.staticField}>{user.email}</p>
                )}
                {editMode && <small>Email cannot be changed</small>}
              </div>
            </div>

            <div className={styles.actions}>
              {!editMode ? (
                <>
                  <button
                    className="button button--primary"
                    onClick={() => setEditMode(true)}
                  >
                    ✏️ Edit Profile
                  </button>
                  <Link to="/dashboard" className="button button--secondary">
                    ← Back to Dashboard
                  </Link>
                </>
              ) : (
                <>
                  <button
                    className="button button--primary"
                    onClick={handleSave}
                  >
                    ✓ Save Changes
                  </button>
                  <button
                    className="button button--secondary"
                    onClick={() => {
                      setEditMode(false);
                      setFormData({
                        fullName: user.fullName,
                        email: user.email,
                      });
                    }}
                  >
                    ✗ Cancel
                  </button>
                </>
              )}
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default ProfilePage;
