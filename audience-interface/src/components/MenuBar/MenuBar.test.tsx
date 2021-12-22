import React from 'react';
import { Button, Grid, Typography } from '@material-ui/core';
import MenuBar from './MenuBar';
import { shallow } from 'enzyme';
import ToggleAudioButton from '../Buttons/ToggleAudioButton/ToggleAudioButton';
import ToggleVideoButton from '../Buttons/ToggleVideoButton/ToggleVideoButton';
import useRoomState from '../../hooks/useRoomState/useRoomState';
import useVideoContext from '../../hooks/useVideoContext/useVideoContext';
import * as utils from '../../utils';

jest.mock('../../hooks/useRoomState/useRoomState');
jest.mock('../../hooks/useVideoContext/useVideoContext');

const mockUseRoomState = useRoomState as jest.Mock<any>;
const mockUseVideoContext = useVideoContext as jest.Mock<any>;

mockUseVideoContext.mockImplementation(() => ({
  isSharingScreen: false,
  room: { name: 'Test Room' },
}));

mockUseRoomState.mockImplementation(() => 'connected');

describe('the MenuBar component', () => {
  beforeEach(() => {
    //@ts-ignore
    utils.isMobile = false;
    process.env.REACT_APP_DISABLE_TWILIO_CONVERSATIONS = 'false';
  });

  it('should disable toggle buttons while reconnecting to the room', () => {
    mockUseRoomState.mockImplementationOnce(() => 'reconnecting');
    const wrapper = shallow(<MenuBar />);
    expect(wrapper.find(ToggleAudioButton).prop('disabled')).toBe(true);
    expect(wrapper.find(ToggleVideoButton).prop('disabled')).toBe(true);
  });

  it('should enable toggle buttons while connected to the room', () => {
    const wrapper = shallow(<MenuBar />);
    expect(wrapper.find(ToggleAudioButton).prop('disabled')).toBe(false);
    expect(wrapper.find(ToggleVideoButton).prop('disabled')).toBe(false);
  });
});
