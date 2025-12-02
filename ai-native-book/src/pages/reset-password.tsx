import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

function ResetPasswordPage() {
  return (
    <Layout title="Reset Password" description="Reset your account password">
      <main style={{ padding: '4rem 0' }}>
        <div className="container" style={{ maxWidth: 640 }}>
          <h1>Reset Password</h1>
          <p>
            Password reset via email will be available soon. In the meantime,
            if you’ve lost access, please contact support or try signing in
            with Google.
          </p>
          <div style={{ display: 'flex', gap: '1rem', marginTop: '1rem' }}>
            <Link className="button button--primary" to="/login">Back to Login</Link>
            <Link className="button button--outline" to="/signup">Create Account</Link>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default ResetPasswordPage;