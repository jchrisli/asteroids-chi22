import React from 'react';

export default function GoForward() {
  //return <img src={process.env.PUBLIC_URL + '/images/up-arrow.png'} alt="" />;
  return (
    <svg width="64" height="64" viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg">
      <rect x="2" y="2" width="60" height="60" rx="9" stroke="#a4bc13" strokeWidth="3" />
      <path d="M31.5 47.9688V17.0312" stroke="#a4bc13" strokeWidth="4" strokeLinecap="round" strokeLinejoin="round" />
      <path
        d="M18.8438 29.6875L31.5 17.0312L44.1562 29.6875"
        stroke="#a4bc13"
        strokeWidth="4"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}
