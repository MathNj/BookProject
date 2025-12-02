import React, { useState } from 'react';
import { useAuth } from '@site/src/contexts/AuthContext';
import LoginModal from './LoginModal';
import SignupModal from './SignupModal';
import styles from './auth.module.css';

export default function AuthButton() {
  const { user, logout } = useAuth();
  const [showLogin, setShowLogin] = useState(false);
  const [showSignup, setShowSignup] = useState(false);

  if (user) {
    return (
      <div className={styles.userContainer}>
        <span className={styles.userName}>{user.name || user.email}</span>
        <button
          className={styles.logoutBtn}
          onClick={logout}
        >
          Sign Out
        </button>
      </div>
    );
  }

  return (
    <>
      <div className={styles.buttonContainer}>
        <button
          className={styles.signInBtn}
          onClick={() => setShowLogin(true)}
        >
          Sign In
        </button>
        <button
          className={styles.signUpBtn}
          onClick={() => setShowSignup(true)}
        >
          Sign Up
        </button>
      </div>

      {showLogin && (
        <LoginModal
          onClose={() => setShowLogin(false)}
          onSignupClick={() => {
            setShowLogin(false);
            setShowSignup(true);
          }}
        />
      )}

      {showSignup && (
        <SignupModal
          onClose={() => setShowSignup(false)}
          onLoginClick={() => {
            setShowSignup(false);
            setShowLogin(true);
          }}
        />
      )}
    </>
  );
}
