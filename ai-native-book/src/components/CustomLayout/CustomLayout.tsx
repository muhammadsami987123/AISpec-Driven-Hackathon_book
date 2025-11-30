import React from 'react';
import Layout from '@theme/Layout';
import { UserNav } from '../components/UserNav/UserNav';

interface CustomLayoutProps {
  children: React.ReactNode;
  title?: string;
  description?: string;
  noFooter?: boolean;
  wrapperClassName?: string;
}

export function CustomLayout({
  children,
  title,
  description,
  noFooter,
  wrapperClassName,
}: CustomLayoutProps) {
  return (
    <Layout
      title={title}
      description={description}
      noFooter={noFooter}
      wrapperClassName={wrapperClassName}
    >
      <style>{`
        .navbar__inner {
          display: flex;
          align-items: center;
          justify-content: space-between;
        }
      `}</style>
      <UserNav />
      {children}
    </Layout>
  );
}

export default CustomLayout;
