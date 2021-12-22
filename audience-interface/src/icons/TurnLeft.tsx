import React from 'react';

export default function TurnLeft() {
  //return <img src={process.env.PUBLIC_URL + '/images/left-arrow.png'} alt="" />;
  return (
    <svg width="64" height="64" viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg">
      <rect width="60" height="60" rx="9" transform="matrix(-1 0 0 1 62 2)" stroke="#a4bc13" strokeWidth="3" />
      <path
        d="M24.0625 35.7187L15.625 27.2812L24.0625 18.8437"
        stroke="#a4bc13"
        strokeWidth="4"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <path
        d="M49.375 44.1562C49.375 39.6807 47.5971 35.3885 44.4324 32.2238C41.2677 29.0591 36.9755 27.2812 32.5 27.2812L15.625 27.2812"
        stroke="#a4bc13"
        strokeWidth="4"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}
