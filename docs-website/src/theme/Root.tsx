/**
 * Swizzled Root Component for Docusaurus
 *
 * This wraps the default Root component from @docusaurus/theme-classic
 * and injects the RAG ChatWidget and Authentication buttons on every page.
 */

import React from 'react';
import RootComponent from '@theme-original/Root';
import ChatWidget from '@site/src/components/ChatWidget';
import AuthButton from '@site/src/components/Auth/AuthButton';
import { AuthProvider } from '@site/src/contexts/AuthContext';
import type RootType from '@theme/Root';
import type { WrapperProps } from '@docusaurus/types';
import styles from './root.module.css';

type RootProps = WrapperProps<typeof RootType>;

export default function Root(props: RootProps): JSX.Element {
  return (
    <AuthProvider>
      <RootComponent {...props} />
      <div className={styles.authContainer}>
        <AuthButton />
      </div>
      <ChatWidget />
    </AuthProvider>
  );
}
