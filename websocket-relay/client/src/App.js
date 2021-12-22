import React from 'react'
import { BrowserRouter, Route, Switch } from "react-router-dom";

import {Typography, AppBar} from '@material-ui/core'
import {makeStyles} from '@material-ui/core/styles'

import Notifications from './components/Notifications'
import Options from './components/Options'

import CreateRoom from "./routes/CreateRoom";
import Room from "./routes/Room";

const useStyles = makeStyles((theme)=> ({
    appBar: {
        borderRadius: 15,
        margin: '30px 100px',
        display: 'flex',
        flexDirection: 'row',
        justifyContent: 'center',
        alignItems: 'center',
        width: '600px',
        border: '2px solid black',
    
        [theme.breakpoints.down('xs')]: {
          width: '90%',
        },
      },
      image: {
        marginLeft: '15px',
      },
      wrapper: {
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        width: '100%',
      },
}));
const App = () => {
    const classes = useStyles();
    return (
        <div className={classes.wrapper}>
        <AppBar className={classes.appBar} position="static" color="inherit">
            <Typography variant="h2" align="center">
            video chat App
            </Typography>
        </AppBar>
        <Options>
            {/* <Notifications/> */}
        </Options>
        <BrowserRouter>
        <Switch>
          <Route path="/" exact component={CreateRoom} />
          <Route path="/room/:roomID" component={Room} />
        </Switch>
      </BrowserRouter>
        </div>
    );
}

export default App;
