import React, { useEffect } from 'react';

export function useUserNav() {
  useEffect(() => {
    const injectUserNav = () => {
      const navbarEnd = document.querySelector('.navbar__items--right');
      if (navbarEnd && !document.getElementById('user-nav-container')) {
        const container = document.createElement('div');
        container.id = 'user-nav-container';
        container.style.display = 'flex';
        container.style.alignItems = 'center';
        container.style.marginLeft = 'auto';
        navbarEnd.appendChild(container);

        // Render UserNav component into the container
        import('./UserNav').then((module) => {
          const { UserNav } = module;
          const root = require('react-dom').createRoot(container);
          root.render(<UserNav />);
        });
      }
    };

    // Wait for DOM to be ready
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', injectUserNav);
      return () => document.removeEventListener('DOMContentLoaded', injectUserNav);
    } else {
      injectUserNav();
    }
  }, []);
}
