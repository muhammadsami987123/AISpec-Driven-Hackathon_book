import React, { useEffect } from 'react';
import NavbarOriginal from '@theme-original/Navbar';

export default function NavbarWrapper(props: any) {
  useEffect(() => {
    // Ensure navbar brand is always visible - FIXED VERSION
    const ensureBrandVisible = () => {
      const brand = document.querySelector('.navbar__brand');
      if (brand) {
        const brandEl = brand as HTMLElement;
        brandEl.style.display = 'flex';
        brandEl.style.visibility = 'visible';
        brandEl.style.opacity = '1';
        brandEl.style.minWidth = 'fit-content';
        brandEl.style.width = 'auto';
        brandEl.style.height = 'auto';
        // CRITICAL: Remove transparent text fill
        brandEl.style.webkitTextFillColor = '';
        brandEl.style.color = '';
      }
      
      const logo = document.querySelector('.navbar__logo');
      if (logo) {
        const logoEl = logo as HTMLElement;
        logoEl.style.display = 'flex';
        logoEl.style.visibility = 'visible';
        logoEl.style.opacity = '1';
        logoEl.style.minWidth = '40px';
        logoEl.style.minHeight = '40px';
      }

      // Find and fix brand text/title - check multiple possible selectors
      const selectors = [
        '.navbar__brand > span',
        '.navbar__brand .navbar__title',
        '.navbar__brand__text',
        '.navbar__brand__link',
        '.navbar__brand a'
      ];
      
      selectors.forEach(selector => {
        const element = document.querySelector(selector);
        if (element) {
          const el = element as HTMLElement;
          el.style.display = 'inline-block';
          el.style.visibility = 'visible';
          el.style.opacity = '1';
          // Only apply gradient to text, not remove visibility
          if (el.textContent && el.textContent.trim()) {
            el.style.webkitTextFillColor = '';
          }
        }
      });

      // Ensure logo image is visible
      const logoImg = document.querySelector('.navbar__logo img, .navbar__logo svg');
      if (logoImg) {
        const imgEl = logoImg as HTMLElement;
        imgEl.style.display = 'block';
        imgEl.style.visibility = 'visible';
        imgEl.style.opacity = '1';
      }
    };

    // Run immediately and after delays to catch all render cycles
    ensureBrandVisible();
    const timer1 = setTimeout(ensureBrandVisible, 50);
    const timer2 = setTimeout(ensureBrandVisible, 200);
    const timer3 = setTimeout(ensureBrandVisible, 500);
    const timer4 = setTimeout(ensureBrandVisible, 1000);
    
    // Also run on DOM mutations
    const observer = new MutationObserver(() => {
      setTimeout(ensureBrandVisible, 10);
    });
    observer.observe(document.body, { childList: true, subtree: true });

    // Run on route changes (for SPA navigation)
    window.addEventListener('popstate', ensureBrandVisible);

    return () => {
      clearTimeout(timer1);
      clearTimeout(timer2);
      clearTimeout(timer3);
      clearTimeout(timer4);
      observer.disconnect();
      window.removeEventListener('popstate', ensureBrandVisible);
    };
  }, []);

  return <NavbarOriginal {...props} />;
}
