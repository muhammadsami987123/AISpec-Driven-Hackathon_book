import React from 'react';
import NavbarItems from '@theme-original/NavbarItem';
import { UserNav } from '../UserNav/UserNav';
import styles from './NavbarItemWrapper.module.css';

export default function NavbarItemWrapper(props) {
  return (
    <>
      <NavbarItems {...props} />
      <UserNav />
    </>
  );
}
