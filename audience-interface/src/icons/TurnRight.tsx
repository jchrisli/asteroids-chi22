import React from 'react';

export default function TurnLeft() {
  //return <img src={process.env.PUBLIC_URL + '/images/right-arrow.png'} alt="" />;
  return (
    <svg width="64" height="64" viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg">
      <rect x="2" y="2" width="60" height="60" rx="9" stroke="#a4bc13" strokeWidth="3" />
      <path
        d="M39.9375 35.7187L48.375 27.2812L39.9375 18.8437"
        stroke="#a4bc13"
        strokeWidth="4"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <path
        d="M14.625 44.1562C14.625 39.6807 16.4029 35.3885 19.5676 32.2238C22.7323 29.0591 27.0245 27.2812 31.5 27.2812L48.375 27.2812"
        stroke="#a4bc13"
        strokeWidth="4"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}
