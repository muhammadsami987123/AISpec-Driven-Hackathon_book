import React, { useEffect, useState } from 'react';
import styles from './UserNav.module.css';

interface User {
  id: string;
  fullName: string;
  email: string;
  image?: string;
}

export function UserNav() {
  const [user, setUser] = useState<User | null>(null);
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Check if user is logged in by checking localStorage
    const token = localStorage.getItem('authToken');
    const savedUser = localStorage.getItem('user');

    if (token && savedUser) {
      try {
        setUser(JSON.parse(savedUser));
      } catch {
        setUser(null);
      }
    }
    setLoading(false);

    // Close dropdown when clicking outside
    const handleClickOutside = (e: MouseEvent) => {
      const userNav = document.querySelector(`.${styles.userNav}`);
      if (userNav && !userNav.contains(e.target as Node)) {
        setIsDropdownOpen(false);
      }
    };

    document.addEventListener('click', handleClickOutside);
    return () => document.removeEventListener('click', handleClickOutside);
  }, []);

  const handleLogout = async () => {
    try {
      const backendUrl = 'http://localhost:5000';
      await fetch(`${backendUrl}/api/auth/logout`, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${localStorage.getItem('authToken')}`,
        },
      });
    } catch (error) {
      console.error('Logout error:', error);
    } finally {
      // Clear local storage
      localStorage.removeItem('authToken');
      localStorage.removeItem('user');
      setUser(null);
      setIsDropdownOpen(false);
      
      // Dispatch auth change event to update navbar
      window.dispatchEvent(new Event('authChange'));
      
      // Redirect to home
      window.location.href = '/';
    }
  };

  if (loading) {
    return null;
  }

  if (!user) {
    return null;
  }

  const initials = user.fullName
    .split(' ')
    .map((n) => n[0])
    .join('')
    .toUpperCase()
    .slice(0, 2);

  return (
    <div className={styles.userNav}>
      <button
        className={styles.profileButton}
        onClick={() => setIsDropdownOpen(!isDropdownOpen)}
        title={user.fullName}
      >
        {user.image ? (
          <img src={user.image} alt={user.fullName} className={styles.profileImage} />
        ) : (
          <div className={styles.avatar}>{initials}</div>
        )}
      </button>

      {isDropdownOpen && (
        <div className={styles.dropdown}>
          <div className={styles.userInfo}>
            <p className={styles.userName}>{user.fullName}</p>
            <p className={styles.userEmail}>{user.email}</p>
          </div>

          <div className={styles.divider} />

          <a
            href="/profile"
            className={styles.menuItem}
            onClick={() => setIsDropdownOpen(false)}
          >
            👤 My Profile
          </a>

          <button
            className={`${styles.menuItem} ${styles.logoutButton}`}
            onClick={handleLogout}
          >
            🚪 Logout
          </button>
        </div>
      )}
    </div>
  );
}
