import React, { useEffect } from 'react';
import { UserNav } from '@site/src/components/UserNav/UserNav';

export default function Root({ children }) {
  useEffect(() => {
    // Check auth status and update DOM
    const updateAuthUI = () => {
      const token = localStorage.getItem('authToken');
      
      if (token) {
        // User is logged in
        document.documentElement.setAttribute('data-user-logged-in', 'true');
        
        // Hide Sign In button
        const signInLink = document.querySelector('.navbar__link[href="/login"]');
        if (signInLink) {
          signInLink.parentElement.style.display = 'none';
        }
      } else {
        // User is logged out
        document.documentElement.removeAttribute('data-user-logged-in');
        
        // Show Sign In button
        const signInLink = document.querySelector('.navbar__link[href="/login"]');
        if (signInLink) {
          signInLink.parentElement.style.display = 'flex';
        }
      }
    };

    // Initial check
    updateAuthUI();

    // Check on auth changes
    window.addEventListener('authChange', updateAuthUI);
    window.addEventListener('storage', updateAuthUI);

    const injectUserNav = () => {
      // Find the navbar items container
      const navbarRight = document.querySelector('.navbar__items--right');
      if (navbarRight && !document.getElementById('user-nav-root')) {
        // Create wrapper div
        const wrapper = document.createElement('div');
        wrapper.id = 'user-nav-root';
        wrapper.style.display = 'flex';
        wrapper.style.alignItems = 'center';
        
        // Append to the end (before GitHub link if needed)
        navbarRight.appendChild(wrapper);

        // Dynamically import and render UserNav
        import('react-dom/client').then(({ createRoot }) => {
          const root = createRoot(wrapper);
          root.render(<UserNav />);
        });
      }
    };

    // Wait a bit for DOM to be fully ready
    const timer = setTimeout(injectUserNav, 100);
    
    // Also try on page navigation
    window.addEventListener('load', () => {
      updateAuthUI();
      injectUserNav();
    });
    
    // Listen for login/logout events to trigger navbar update
    const handleAuthChange = () => {
      updateAuthUI();
      injectUserNav();
    };
    
    window.addEventListener('authChange', handleAuthChange);
    
    return () => {
      clearTimeout(timer);
      window.removeEventListener('load', injectUserNav);
      window.removeEventListener('authChange', handleAuthChange);
      window.removeEventListener('storage', updateAuthUI);
      window.removeEventListener('authChange', updateAuthUI);
    };
  }, []);

  return <>{children}</>;
}
